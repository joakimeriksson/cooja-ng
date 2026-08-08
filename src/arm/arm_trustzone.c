/*
 * ARMv8-M Security Extension (TrustZone-M) — security attribution engine.
 *
 * SAU + IDAU address attribution per the ARMv8-M ARM (DDI 0553). The SAU +
 * IDAU combination follows the QEMU `v8m_security_lookup` model so that QEMU
 * and Renode remain valid functional oracles for differential testing
 * (docs/design/trustzone-m-plan.md, Phase 1).
 *
 * Scope note: this compilation unit is the pure attribution *engine*. Wiring
 * it into the load/store hot path (enforcement) and into the memory-mapped
 * SAU_* registers happens in later increments; keeping the engine standalone
 * lets it be unit-tested directly with no way yet to enter Secure state.
 */
#include "arm_trustzone.h"
#include "arm_cpu.h"

/*
 * SAU-only attribution.
 *
 *  - SAU disabled: the whole space is Secure, unless SAU_CTRL.ALLNS makes it
 *    all Non-secure. (No region is NSC when the SAU is disabled.)
 *  - SAU enabled: search the enabled regions. A single matching region makes
 *    the address Non-secure, or NSC if the region's NSC bit is set. Zero
 *    matches => Secure. More than one match => Secure (and region invalid),
 *    matching hardware's "overlapping regions are treated as Secure".
 *
 * `*ns` = address is Non-secure; `*nsc` = address is Non-secure-callable
 * (which is Secure memory, so *ns stays false in that case).
 */
void arm_sau_check(const arm_cpu_t *cpu, uint32_t addr, bool *ns, bool *nsc)
{
    *ns = false;
    *nsc = false;

    if (!(cpu->sau_ctrl & ARM_SAU_CTRL_ENABLE)) {
        /* Disabled: ALLNS decides. */
        *ns = (cpu->sau_ctrl & ARM_SAU_CTRL_ALLNS) != 0;
        return;
    }

    bool matched = false;
    for (unsigned r = 0; r < cpu->sau_sregions && r < 8; r++) {
        if (!(cpu->sau_rlar[r] & ARM_SAU_RLAR_ENABLE))
            continue;
        /* Base is 32-byte aligned (bits 4:0 == 0); limit's low 5 bits read
         * as 1, so the region spans [base, limit|0x1f] inclusive. */
        uint32_t base  = cpu->sau_rbar[r] & ~0x1fu;
        uint32_t limit = (cpu->sau_rlar[r] & ~0x1fu) | 0x1fu;
        if (addr < base || addr > limit)
            continue;

        if (matched) {
            /* Overlapping match => Secure, and the earlier decision is void. */
            *ns = false;
            *nsc = false;
            return;
        }
        matched = true;
        if (cpu->sau_rlar[r] & ARM_SAU_RLAR_NSC)
            *nsc = true;   /* Secure, callable */
        else
            *ns = true;    /* Non-secure */
    }
}

/* Default IDAU: attribute nothing — leave the decision to the SAU. */
arm_idau_result_t arm_idau_check_default(uint32_t addr)
{
    (void)addr;
    arm_idau_result_t r = { .ns = true, .nsc = false, .exempt = false };
    return r;
}

/*
 * Combine SAU and IDAU. The IDAU can only ever make an address *more* secure:
 * if the IDAU says Secure (idau.ns == false), the result is Secure, and the
 * NSC attribute survives only if the IDAU also permits it. This mirrors the
 * QEMU combination and the ARM ARM `SecurityCheck` pseudocode.
 */
arm_sec_attr_t arm_security_attr(const arm_cpu_t *cpu, uint32_t addr)
{
    if (!cpu->tz_enabled)
        return ARM_SEC_NONSECURE;

    bool ns, nsc;
    arm_sau_check(cpu, addr, &ns, &nsc);

    arm_idau_result_t idau = arm_idau_check_default(addr);

    if (idau.exempt)
        return ARM_SEC_SECURE;

    if (!idau.ns) {
        /* IDAU forces Secure. */
        ns = false;
        if (!idau.nsc)
            nsc = false;
    }

    if (ns)
        return ARM_SEC_NONSECURE;
    return nsc ? ARM_SEC_NSC : ARM_SEC_SECURE;
}

bool arm_mem_access_permitted(const arm_cpu_t *cpu, uint32_t addr)
{
    /* No extension, or running Secure: every address is reachable. */
    if (!cpu->tz_enabled || cpu->secure)
        return true;
    /* Non-secure: only Non-secure memory is reachable. NSC and Secure are
     * both Secure memory for data purposes, so both are refused. */
    return arm_security_attr(cpu, addr) == ARM_SEC_NONSECURE;
}

void arm_record_secure_fault(arm_cpu_t *cpu, uint32_t addr)
{
    cpu->sfsr |= ARM_SFSR_AUVIOL | ARM_SFSR_SFARVALID;
    cpu->sfar = addr;
    cpu->secure_fault_pending = true;
}

int arm_sau_region(const arm_cpu_t *cpu, uint32_t addr)
{
    if (!(cpu->sau_ctrl & ARM_SAU_CTRL_ENABLE))
        return -1;
    int found = -1;
    for (unsigned r = 0; r < cpu->sau_sregions && r < 8; r++) {
        if (!(cpu->sau_rlar[r] & ARM_SAU_RLAR_ENABLE))
            continue;
        uint32_t base  = cpu->sau_rbar[r] & ~0x1fu;
        uint32_t limit = (cpu->sau_rlar[r] & ~0x1fu) | 0x1fu;
        if (addr >= base && addr <= limit) {
            if (found >= 0)
                return -1;   /* overlap => region invalid */
            found = (int)r;
        }
    }
    return found;
}

/* TT response bit positions (ARMv8-M). */
#define ARM_TT_SREGION_SHIFT 8
#define ARM_TT_SRVALID  (1u << 17)
#define ARM_TT_R        (1u << 18)
#define ARM_TT_RW       (1u << 19)
#define ARM_TT_S        (1u << 22)

uint32_t arm_tt_response(const arm_cpu_t *cpu, uint32_t addr, bool alt)
{
    (void)alt;  /* alternate-domain (TTA/TTAT) refinement deferred */
    uint32_t resp = 0;

    arm_sec_attr_t attr = arm_security_attr(cpu, addr);
    if (attr != ARM_SEC_NONSECURE)
        resp |= ARM_TT_S;   /* Secure or NSC */

    int region = arm_sau_region(cpu, addr);
    if (region >= 0) {
        resp |= ((uint32_t)region << ARM_TT_SREGION_SHIFT) & 0xFF00u;
        resp |= ARM_TT_SRVALID;
    }

    /* Without an MPU model, memory is treated as read/write accessible. */
    resp |= ARM_TT_R | ARM_TT_RW;
    return resp;
}
