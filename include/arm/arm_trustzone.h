/*
 * ARMv8-M Security Extension (TrustZone-M) — security attribution.
 *
 * Implements the SAU + IDAU address security attribution defined by the
 * ARMv8-M Architecture Reference Manual (DDI 0553), combination rules
 * matching the QEMU/Renode model so those emulators can serve as functional
 * oracles. See docs/design/trustzone-m-plan.md (Phase 1).
 */
#ifndef ARM_TRUSTZONE_H
#define ARM_TRUSTZONE_H

#include <stdbool.h>
#include <stdint.h>

struct arm_cpu;
typedef struct arm_cpu arm_cpu_t;

/* Security attribute of an address, most-secure last. NSC ("non-secure
 * callable") is Secure memory from which SG veneer entries are permitted. */
typedef enum {
    ARM_SEC_NONSECURE = 0,   /* Non-secure */
    ARM_SEC_NSC       = 1,   /* Secure, Non-secure-callable (holds SG veneers) */
    ARM_SEC_SECURE    = 2,   /* Secure */
} arm_sec_attr_t;

/* SAU_CTRL / SAU_RLAR bit fields. */
#define ARM_SAU_CTRL_ENABLE   (1u << 0)
#define ARM_SAU_CTRL_ALLNS    (1u << 1)
#define ARM_SAU_RLAR_ENABLE   (1u << 0)
#define ARM_SAU_RLAR_NSC      (1u << 1)

/* SecureFault Status Register (SFSR) bit fields. */
#define ARM_SFSR_INVEP     (1u << 0)  /* invalid secure-gateway entry point */
#define ARM_SFSR_INVIS     (1u << 1)  /* invalid integrity signature */
#define ARM_SFSR_INVER     (1u << 2)  /* invalid exception return */
#define ARM_SFSR_AUVIOL    (1u << 3)  /* attribution-unit violation (data) */
#define ARM_SFSR_INVTRAN   (1u << 4)  /* invalid transition */
#define ARM_SFSR_LSPERR    (1u << 5)  /* lazy state preservation error */
#define ARM_SFSR_SFARVALID (1u << 6)  /* SFAR holds a valid address */
#define ARM_SFSR_LSERR     (1u << 7)  /* lazy state error */

/* Result of an IDAU attribution check for one address. */
typedef struct {
    bool ns;        /* IDAU considers the address non-secure */
    bool nsc;       /* IDAU marks it non-secure-callable */
    bool exempt;    /* address is exempt from security checking (always Secure-ish) */
} arm_idau_result_t;

/* Full security attribution for `addr`, combining SAU and the SoC IDAU with
 * the conservative rule (Secure if either unit says Secure). Returns
 * ARM_SEC_NONSECURE unconditionally when the SoC has no security extension. */
arm_sec_attr_t arm_security_attr(const arm_cpu_t *cpu, uint32_t addr);

/* SAU-only attribution (no IDAU), exposed for unit testing and for use by
 * arm_security_attr(). Sets *ns and *nsc per SAU_CTRL and the enabled
 * regions selected by SAU_RNR/RBAR/RLAR. */
void arm_sau_check(const arm_cpu_t *cpu, uint32_t addr, bool *ns, bool *nsc);

/* Index of the single enabled SAU region containing `addr`, or -1 if none
 * (or if the SAU is disabled / multiple regions overlap). Used by TT. */
int arm_sau_region(const arm_cpu_t *cpu, uint32_t addr);

/* Build the ARMv8-M TT/TTT/TTA/TTAT response word for `addr`. `alt` selects
 * the alternate (Non-secure) domain view (TTA/TTAT), only meaningful from
 * Secure state. Reports the Secure (S), SAU region, and R/RW attributes. */
uint32_t arm_tt_response(const arm_cpu_t *cpu, uint32_t addr, bool alt);

/* Default IDAU: everything non-secure, nothing exempt (i.e. attribution is
 * left entirely to the SAU). SoCs with a hardware IDAU (nRF54L15 SPU) install
 * their own check; until then this is the behaviour. */
arm_idau_result_t arm_idau_check_default(uint32_t addr);

/* Data-access enforcement (Step 2). True if the current security state may
 * access `addr` for data: Non-secure may reach only Non-secure memory; Secure
 * may reach anything; always true when the SoC has no security extension. */
bool arm_mem_access_permitted(const arm_cpu_t *cpu, uint32_t addr);

/* Record an attribution-unit SecureFault (SFSR.AUVIOL) for `addr`: latches
 * SFSR/SFAR and marks the fault pending. The exception is taken later by the
 * secure exception model (Step 4). */
void arm_record_secure_fault(arm_cpu_t *cpu, uint32_t addr);

#endif /* ARM_TRUSTZONE_H */
