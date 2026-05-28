# Bug Fix: LDRD/STRD Immediate Offset Double-Shift in ARM Thumb-2 Emulator

## Summary

The ARM emulator applied the `imm8 × 4` word-to-byte-offset scaling **twice** for
`LDRD` and `STRD` immediate-offset instructions, producing offsets 4× too large
(`field × 16` instead of `field × 4`).  Instructions with `imm8 = 0` were
unaffected.  All other instructions sharing the same decode block (STREX, LDREX,
TBB/TBH) computed their offsets correctly.

The symptom in nRF52840 firmware was a **startup hang**: the node printed its
link-layer address then froze indefinitely before printing the tentative IPv6
address.  Root cause: a `STRD` in `snprintf` with `imm8 = 2` was writing to
`[sp+32]` instead of `[sp+8]`, corrupting the `snprintf` output-buffer context
and causing the `buffer_str` callback to attempt a 4 GB `memcpy` loop.

---

## ARM Architecture Background

**Thumb-2 LDRD/STRD immediate encoding (T1):**

```
1110 100P U1W1 Rn  |  Rt  Rt2  imm8
```

The `imm8` field is a **word offset** — the actual byte offset is `imm8 × 4`.
The same field is used for STREX and LDREX T1, with the same `× 4` scaling.

So for `strd r4, r3, [sp, #8]` the instruction encodes `imm8 = 2` (the assembler
divides 8 by 4), and the emulator must recover the byte offset as `2 × 4 = 8`.

---

## The Bug

In `src/arm/arm_cpu.c`, the decode block for
*Load/store dual, exclusive, table branch* (`op1 == 1`) extracts `imm8` at the
top of the block, then uses it in multiple sub-cases:

```c
// Line ~1754 — shared extraction at top of block
uint32_t imm8 = (hw2 & 0xFF) << 2;   // ← pre-scales field by 4
```

This pre-scaling is **correct** for STREX and LDREX, which use `imm8` directly:

```c
// STREX (line ~1826)
uint32_t addr = cpu->reg[rn] + imm8;  // byte addr = Rn + field×4 ✓

// LDREX (line ~1842)
uint32_t addr = cpu->reg[rn] + imm8;  // byte addr = Rn + field×4 ✓
```

However, the LDRD/STRD sub-case (and the LDRD-literal sub-case) applies the
`× 4` scaling **a second time**:

```c
// LDRD/STRD (line ~1846)
uint32_t offset = imm8 << 2;   // ← shifts again: field×4×4 = field×16 ✗

// LDRD literal (line ~1817)
uint32_t off = imm8 << 2;      // ← same double-shift ✗
```

### Concrete Example

`strd r4, r3, [sp, #8]`  — encoding: `e9cd 4302`

| Step | Value |
|------|-------|
| `hw2 & 0xFF` | `0x02` (the `imm8` field) |
| `imm8 = field << 2` | `0x08` (= 8) |
| **old** `offset = imm8 << 2` | `0x20` (= **32**) — writes to `[sp+32]` ✗ |
| **new** `offset = imm8` | `0x08` (= **8**) — writes to `[sp+8]` ✓ |

The correct byte offset is 8.  The bug produced 32.

---

## How the Bug Was Discovered

### Symptom

nRF52840 Contiki-NG firmware (c509-hw-test example) hung at startup with a
speed ratio of ~0.1× real-time (both nodes spinning in a tight loop).  Debug
`printf` calls narrowed the hang to:

```
snprintf(buf, 40, "%x", 0xfe80)
```

called from `uiplib_ipaddr_snprint` during IPv6 address printing at boot.

### Root Cause Trace

`snprintf` in the Newlib-nano stdlib sets up a `buffer_str` callback context on
the stack:

```
snprintf stack frame (sp relative, after push+sub):
  [sp+ 8] = output buffer ptr    ← intended STRD target (Rt  = r4 = buf)
  [sp+12] = remaining capacity   ← intended STRD target (Rt2 = r3 = n)
  [sp+16] = buffer_str callback
  [sp+20] = &[sp+8]              (context pointer passed to format_str_v)
  ...
  [sp+32] = saved r4             ← actual STRD write target with double-shift
  [sp+36] = saved lr             ← actual STRD+4 write target with double-shift
```

With the double-shift bug:
- `[sp+8]` (buf ptr) remained uninitialized (zero from BSS/stack init)
- `[sp+12]` (remaining = 40) was never written

When `buffer_str` ran it read `remaining = context[1] = [sp+12] = 0`.  The
`buffer_str` implementation computes `copy_len = remaining - 1` when
`remaining ≤ requested_len`, giving `copy_len = 0 - 1 = 0xFFFFFFFF`.  The
subsequent `memcpy(buf, src, 0xFFFFFFFF)` became the observed infinite loop.

### Why the Bug Was Hidden Before

The very same `STRD` instruction with `imm8 = 2` existed in the firmware before
the `imm8 << 2` fix was introduced.  In the **original** emulator state
(`offset = raw imm8 = 2`), the STRD wrote to `[sp+2]` instead of `[sp+8]`.
That was also wrong, but `[sp+2]` happened to land inside uninitialized local
storage, and whatever garbage was already at `[sp+8]` / `[sp+12]` from previous
stack frames produced a non-zero `remaining` value, so `buffer_str` did not
0-wrap.  The actual `snprintf` output was corrupt (written to the wrong address),
but execution continued without hanging.

The `imm8 << 2` fix was added to correct the offset calculation, but it was
applied in **addition** to the existing pre-shift on line 1754, creating the
double-shift and a regression from "wrong address but non-crashing" to "correct
address but crashing".

---

## The Fix

Keep `imm8 = (hw2 & 0xFF) << 2` at the shared extraction point (correct for
STREX/LDREX/LDAEX/STLEX which use the value directly).  Remove the redundant
`<< 2` from the two LDRD/STRD sub-cases so they use `imm8` directly (which
already represents the byte offset):

```c
// LDRD literal  — was: off = imm8 << 2
uint32_t off = imm8;

// LDRD / STRD   — was: offset = imm8 << 2
uint32_t offset = imm8;
```

### Verification

| Instruction | `imm8` field | Before fix (offset) | After fix (offset) | Correct |
|-------------|-------------|--------------------|--------------------|---------|
| `strd r4, r3, [sp, #8]` | 2 | 32 | **8** | 8 |
| `ldrd r3, r0, [sl]` | 0 | 0 | **0** | 0 |
| `strd r2, r3, [r4]` | 0 | 0 | **0** | 0 |
| `strd r2, r4, [r9, #4]` | 1 | 16 | **4** | 4 |
| `strex rd, rt, [rn, #8]` | 2 | 8 ✓ | 8 ✓ | 8 |
| `ldrex rt, [rn, #4]` | 1 | 4 ✓ | 4 ✓ | 4 |

Instructions with `imm8 = 0` (the vast majority in practice) were unaffected
by either the original bug or this fix.

---

## Test Results

After the fix, nRF52840 Contiki-NG boot completes normally:

```
[INFO: Main] Tentative link-local IPv6 address: fe80::f6ce:3600:0:1
[INFO: DTLS] Initializing DTLS support with library "Mbed TLS 3.6.2"
[INFO: C509 Server] DTLS server ready
...
[INFO: DTLS] Sent DTLS message of len = 93    ← ClientHello from Node 2
```

- Speed ratio: **5.0× real-time** (was 0.1× with infinite loop)
- RF frames exchanged: DTLS ClientHello + ACK visible within 15 s simulation

---

## Files Changed

| File | Change |
|------|--------|
| `src/arm/arm_cpu.c` | Line ~1817: `off = imm8` (was `imm8 << 2`) |
| `src/arm/arm_cpu.c` | Line ~1846: `offset = imm8` (was `imm8 << 2`) |
| `src/arm/arm_cpu.c` | Updated comments to clarify `imm8` is the byte offset |

---

## Affected Instruction Classes

| Instruction | Encoding | imm8 used as | Status before fix | Status after fix |
|-------------|----------|--------------|-------------------|-----------------|
| LDRD (immediate) | T1 | byte offset (= field×4) | ×16 ✗ | ×4 ✓ |
| STRD (immediate) | T1 | byte offset (= field×4) | ×16 ✗ | ×4 ✓ |
| LDRD (literal, PC-relative) | T1 | byte offset (= field×4) | ×16 ✗ | ×4 ✓ |
| STREX | T1 | byte offset (= field×4) | ×4 ✓ | ×4 ✓ |
| LDREX | T1 | byte offset (= field×4) | ×4 ✓ | ×4 ✓ |
| LDAEX/STLEX | T1 | direct addr | n/a | n/a |
| TBB/TBH | — | index (not offset) | n/a | n/a |
