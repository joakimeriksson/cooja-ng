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

/* Default IDAU: everything non-secure, nothing exempt (i.e. attribution is
 * left entirely to the SAU). SoCs with a hardware IDAU (nRF54L15 SPU) install
 * their own check; until then this is the behaviour. */
arm_idau_result_t arm_idau_check_default(uint32_t addr);

#endif /* ARM_TRUSTZONE_H */
