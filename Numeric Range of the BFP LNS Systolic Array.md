# Numeric Range of the BFP/LNS Systolic Array

*Vivek · 22 Oct 2026*

**Contents**

- [Range of everything](#range-of-everything)
- [Names](#names)
- [Step 1: the number v](#step-1-the-number-v)
- [Step 2: the input block](#step-2-the-input-block)
- [Step 3: into the PE](#step-3-into-the-pe)
- [Step 4: the multiply, in log form](#step-4-the-multiply-in-log-form)
- [Step 5: back to a normal number (antilog and shift)](#step-5-back-to-a-normal-number-antilog-and-shift)
- [Step 6: the sum](#step-6-the-sum)
- [Step 7: the output block](#step-7-the-output-block)
- [Where the error comes from](#where-the-error-comes-from)
- [What the driver should check](#what-the-driver-should-check)

## Range of everything

The table lists every quantity in the design, from input to output: its allowed range, its value at the default settings, and what happens if it goes outside. The names are defined in the next section, and each row is derived in the steps after it.

| Quantity | Allowed range (formula) | Default value | If outside the range |
| --- | --- | --- | --- |
| Block exponent E | -2^(EXP\_WIDTH-1) to 2^(EXP\_WIDTH-1) - 1 | -128 to 127 | output: E clamped, exp\_overflow or exp\_underflow = 1 |
| Size of any input or result | about 2^E\_MIN to 2^E\_MAX | about 10^-38 to 10^38 | same as E |
| Residual L (stored integer) | L\_MIN to L\_MAX | -32 to 31 | saturates at the end |
| Scale | 0 to SCALE\_MAX | 0 to 15 | saturates at 15 |
| Step (value of one L unit) | 2^scale / 2^FRAC\_WIDTH octave | 1/256 octave at scale 0 | - |
| Rounding error per stored number | at most 2^(step/2) - 1 | 0.14% at scale 0, 4.4% at scale 5, 9% at scale 6 | - |
| Spread of the 4 numbers in one block (for at most 4.4% each) | farthest one at most L\_MAX x step from E, at scale 5 | 36x (one outlier) to 215x (evenly spread) | coarser rounding, no flag |
| Scale the PE can take in | scale at most MANTISSA\_WIDTH - BLOCK\_MANTISSA\_WIDTH | at most 10 | the log value wraps, no flag |
| PE exponent E\_row + E\_col | EXP\_WIDTH + 1 bits | -256 to 254 | cannot happen |
| One product term, compared with 2^(E\_row + E\_col) | L\_row + L\_col below DATA\_WIDTH - FRAC\_WIDTH | below 2^24 | top bits lost, no flag |
| Sum of ARRAY\_SIZE terms, compared with 2^(E\_row + E\_col) | L\_row + L\_col below DATA\_WIDTH - FRAC\_WIDTH - log2(ARRAY\_SIZE) | below 2^22 (about 4 million) | top bits lost, no flag |
| Smallest product term that survives | L\_row + L\_col at least -FRAC\_WIDTH | at least 2^-8 | term becomes 0 |
| Error added by the output encoder's log | at most 0.086 octave | at most 6.2% | always present |

## Names

Parameters keep their exact RTL names. One **octave** means one doubling, i.e. a difference of 1 in log2: 1 to 8 is 3 octaves.

**Design parameters** (the Makefile knobs)

| Name | Meaning | Default |
| --- | --- | --- |
| EXP\_WIDTH | bits of the block exponent E (signed) | 8 |
| BLOCK\_MANTISSA\_WIDTH | bits of each number's residual L (signed) | 6 |
| SCALE\_FACTOR\_WIDTH | bits of the block's scale | 4 |
| FRAC\_WIDTH | fraction bits of every log value | 8 |
| MANTISSA\_WIDTH | bits of the log bus into the PE | 16 |
| DATA\_WIDTH | bits of the PE's product; the accumulator has DATA\_WIDTH + 1 | 32 |
| ARRAY\_SIZE | numbers per block = rows and columns of the PE array | 4 |

**Short names derived from them**

| Name | Equals | Default |
| --- | --- | --- |
| E\_MIN, E\_MAX | -2^(EXP\_WIDTH-1), 2^(EXP\_WIDTH-1) - 1 | -128, 127 |
| L\_MIN, L\_MAX | -2^(BLOCK\_MANTISSA\_WIDTH-1), 2^(BLOCK\_MANTISSA\_WIDTH-1) - 1 | -32, 31 |
| SCALE\_MAX | 2^SCALE\_FACTOR\_WIDTH - 1 | 15 |

**Symbols** (each is introduced again where it is first used)

| Name | Meaning |
| --- | --- |
| v | one real number |
| E | the exponent shared by a block |
| L | one number's residual, the integer stored in the block |
| scale | the block's scale, an integer |
| step | what one unit of L is worth, in octaves |
| E\_row, E\_col, L\_row, L\_col | E and L of the row and column numbers that meet in one PE |
| k, f | the integer and fraction parts of L\_row + L\_col |

## Step 1: the number v

The design never stores v directly. It stores the sign, a zero flag and log2 of the size:

```math
v = (-1)^{sign} \cdot 2^{\log_2 |v|}
```

- **sign** = 1 if v is negative. It is kept as one bit because a log has no sign.
- **zero** = 1 if v is exactly 0. It is kept as one bit because log2(0) does not exist.
- **log2|v|** carries everything else. A multiply becomes an add: log2|a x b| = log2|a| + log2|b|. That is the whole reason for working in logs.

**Example:** v = -20 gives sign = 1, zero = 0, log2|v| = 4.32.

**Assumption:** from here on v is not zero; zero elements just carry their flag through and are skipped in every sum.

## Step 2: the input block

A block packs ARRAY\_SIZE numbers. Their logs are usually close, so the common part is stored once as E and each number keeps only a small leftover L. This is what lets L be just a few bits.

We follow one block through every sub-step, the 4 numbers **20, 5, 3, 1**, with the default widths (FRAC\_WIDTH = 8, BLOCK\_MANTISSA\_WIDTH = 6).

### 2a. The shared exponent E

**What it is.** One integer per block that says roughly how big the block's numbers are.

**How it is chosen.** E is the average of the logs of the non-zero numbers, rounded to a whole number:

```math
E = \mathrm{round}\Big(\frac{1}{n}\sum_{\text{non-zero } v} \log_2|v|\Big), \qquad n = \text{number of non-zero elements}
```

*Example:* the logs are log2(20) = 4.322, log2(5) = 2.322, log2(3) = 1.585, log2(1) = 0. Their average is 8.229 / 4 = 2.057, so E = 2.

**Why the average and not the largest log.** Each number will store only its distance from E, and that distance is signed. With E at the average, some distances are positive and some negative, so both halves of the signed field are used. With E at the largest log, every distance would be 0 or negative and the positive half of the field would be wasted. In the example the distances are +2.32, +0.32, -0.42 and -2: both signs are used.

**Why zeros are left out.** log2(0) is minus infinity, which would drag the average down without limit. A zero is carried by its zero flag instead.

**Its range.** E is a signed integer of EXP\_WIDTH bits, so E\_MIN ≤ E ≤ E\_MAX (-128 to 127 by default).

**One side effect.** Rounding to a whole number can put E up to 0.5 octave away from the true average (2 instead of 2.057 here). This matters in 2g.

### 2b. What is left for each number

The block rebuilds a number as v = 2^E x 2^(leftover). So the leftover is everything E does not cover:

```math
\log_2|v| = E + L_{true} \quad\Longrightarrow\quad L_{true} = \log_2|v| - E
```

L\_true is a real number, measured in octaves.

*Example (E = 2):* 20 → 4.322 - 2 = **2.322**; 5 → **0.322**; 3 → **-0.415**; 1 → **-2**.

### 2c. Storing the leftover as a small integer

The leftover has to be stored as the integer L, which has only BLOCK\_MANTISSA\_WIDTH bits.

**Step 1: hardware units.** All logs in the design are fixed point with FRAC\_WIDTH fraction bits. The smallest unit is therefore 1/2^FRAC\_WIDTH octave (1/256 by default), and a leftover in hardware units is L\_true x 2^FRAC\_WIDTH:

- 20: 2.322 x 256 = **594**
- 5: **82**
- 3: **-106**
- 1: **-512**

**Step 2: these do not fit.** A signed integer of BLOCK\_MANTISSA\_WIDTH bits only holds L\_MIN to L\_MAX, i.e. -32 to +31 at 6 bits. 594 is far too big.

**Step 3: shrink by a power of two.** The value is divided by 2^scale and rounded:

```math
L = \mathrm{round}\Big(\frac{L_{true} \cdot 2^{FRAC\_WIDTH}}{2^{scale}}\Big)
```

It has to be a power of two because dividing by 2^scale is just a right shift, and multiplying back later (Step 3) is just a left shift. Both cost almost nothing in hardware; any other divisor would need a real divider and multiplier.

**Step 4: what one unit of L is worth.** After the shrink, one count of L stands for 2^scale hardware units, and each hardware unit is 1/2^FRAC\_WIDTH octave. Multiplying the two gives the value of one count of L in octaves, called the **step**:

```math
step = 2^{scale} \times \frac{1}{2^{FRAC\_WIDTH}} = 2^{\,scale - FRAC\_WIDTH} \text{ octave}, \qquad L \approx \frac{L_{true}}{step}
```

At scale 5: step = 2^(5-8) = 1/8 octave.

**Why the name.** L can only be a whole number, so the leftovers the block can store are 0, ±1 step, ±2 steps, and so on, like the stairs of a staircase. The step is the height of one stair, the gap between two neighbouring storable values. At step 1/8:

```
L       :   17      18      19      20
octaves :  2.125   2.250   2.375   2.500
```

The true 2.322 falls between two stairs, 2.25 and 2.375, and is rounded to one of them.

### 2d. Choosing the scale

**The rule.** The encoder tries scale = 0, 1, 2, ... in turn and takes the **first** one for which every non-zero number of the block gets an L inside L\_MIN..L\_MAX.

**Why the smallest.** The smallest scale gives the smallest step, so the finest staircase and the smallest rounding error (2e). A bigger scale would also fit, but it would round more coarsely for no benefit.

*Example:* the hardest number is 20 (594 units):

| scale | 594 / 2^scale | rounded L | fits in -32..31? |
| --- | --- | --- | --- |
| 3 | 74.3 | 74 | no |
| 4 | 37.1 | 37 | no |
| 5 | 18.6 | 19 | **yes** |

At scale 5 the others are 82/32 = 2.6 → 3, -106/32 = -3.3 → -3, and -512/32 = -16. All fit, so **scale = 5** and the block stores L = 19, 3, -3, -16.

**The field is not symmetric.** A signed field reaches one unit further on the negative side (-32) than on the positive side (+31). So the farthest a number can sit **above** E is L\_MAX x step, and the farthest **below** E is abs(L\_MIN) x step. This is called the **reach**.

**If nothing fits.** If even SCALE\_MAX does not fit, the scale stays at SCALE\_MAX and the out-of-range L is clipped to L\_MIN or L\_MAX. That number is then badly wrong.

### 2e. How much error the rounding costs

This is derived in two parts: first the error in the log, then the error in the value.

**Part 1: the error in the log is at most half a step.**

1. Only whole multiples of the step can be stored: ..., 18 x step, 19 x step, ...
2. The true leftover almost always falls **between** two neighbouring stored values. For the 20: 2.322 lies between 2.25 (L = 18) and 2.375 (L = 19).
3. Its distance to the left neighbour plus its distance to the right neighbour is exactly one step, because the two neighbours are one step apart. Here: 0.072 + 0.053 = 0.125 = 1/8.
4. Rounding picks the **nearer** neighbour, so the error is the smaller of two distances that add up to one step.
5. The smaller of two numbers with a fixed total is never more than half the total. So, calling the error in the log e:

```math
|e| \le \frac{step}{2}
```

The worst case is a true value exactly in the middle of two stairs: both distances are then step/2.

*Example:* 2.322 is rounded to 2.375, so e = 0.053, below the limit 1/16 = 0.0625.

```
 2.250                2.375
   |----------x---------|
   L = 18   2.322      L = 19
   |<-0.072->|<-0.053-->|
```

**Part 2: from the log error to the value error.**

6. The stored log is the true log plus e: log2(v\_stored) = log2(v) + e.
7. Undo the log by raising 2 to both sides, using 2^(a + b) = 2^a x 2^b:

```math
v_{stored} = 2^{\log_2 v} \cdot 2^{e} = v \cdot 2^{e}
```

So an error **added** to the log becomes an error **multiplied** onto the value.

8. The relative error is therefore:

```math
\frac{v_{stored} - v}{v} = \frac{v \cdot 2^{e} - v}{v} = 2^{e} - 1
```

9. It is largest when e is largest, e = step/2:

```math
\frac{|v_{stored} - v|}{|v|} \;\le\; 2^{\,step/2} - 1
```

(For a negative e the error is 1 - 2^(-step/2), which is slightly smaller, so this bound covers both.)

*Example:* e = 0.053 gives 2^0.053 - 1 = 3.7%, and indeed 2^(2 + 19/8) = 20.75 is 3.7% above 20. The bound at step 1/8 is 2^(1/16) - 1 = 4.4%.

**The whole block:**

| v | L\_true (octaves) | hardware units | L at scale 5 | stored value | error |
| --- | --- | --- | --- | --- | --- |
| 20 | 2.322 | 594 | 19 | 2^(2 + 19/8) = 20.75 | +3.7% |
| 5 | 0.322 | 82 | 3 | 2^(2 + 3/8) = 5.19 | +3.7% |
| 3 | -0.415 | -106 | -3 | 2^(2 - 3/8) = 3.08 | +2.8% |
| 1 | -2 | -512 | -16 | 2^(2 - 2) = 1 | 0% |

All four are inside the 4.4% bound.

### 2f. Step, error and reach for every scale

Each row uses the formulas above with FRAC\_WIDTH = 8: step = 2^(scale - 8), error = 2^(step/2) - 1, reach above E = L\_MAX x step.

| scale | step (octave) | max error per number | reach, 6-bit L (31 x step) | reach, 8-bit L (127 x step) |
| --- | --- | --- | --- | --- |
| 0 | 0.0039 | 0.14% | 0.12 | 0.50 |
| 1 | 0.0078 | 0.27% | 0.24 | 0.99 |
| 2 | 0.0156 | 0.54% | 0.48 | 1.98 |
| 3 | 0.031 | 1.09% | 0.97 | 3.97 |
| 4 | 0.0625 | 2.19% | 1.94 | 7.94 |
| 5 | 0.125 | 4.43% | 3.88 | 15.9 |
| 6 | 0.25 | 9.05% | 7.75 | 31.8 |
| 7 | 0.5 | 18.9% | 15.5 | 63.5 |

The trade-off is visible in every row: each step up in scale doubles the reach and roughly doubles the error.

### 2g. How far apart the numbers of one block may be

**The question.** How different can the numbers sharing one block be before the error gets too big?

**Part 1: what has to fit.** From 2d, every number must be within reach of E: within L\_MAX x step above it and abs(L\_MIN) x step below it. So the number **farthest from E** decides the scale for the whole block, and with it the error of all four numbers.

**Part 2: how far the farthest number is.** Define the **spread** as the largest log minus the smallest log of the block, in octaves. E is the average of the logs, so it sits somewhere inside the spread, and where exactly depends on how the numbers are arranged.

*Case A, evenly spread:* the numbers 1, 2, 4, 8 have logs 0, 1, 2, 3, so spread = 3. The average is (0 + 1 + 2 + 3) / 4 = 1.5. The farthest numbers, 1 and 8, are each 1.5 away, i.e. **spread / 2**.

*Case B, one outlier:* the numbers 1, 1, 1, 8 have logs 0, 0, 0, 3, so spread = 3 again. Now the average is (0 + 0 + 0 + 3) / 4 = 0.75, because the three equal numbers pull it toward them. The outlier 8 is 3 - 0.75 = 2.25 away, i.e. **3/4 of the spread**.

*Case B in general:* take ARRAY\_SIZE - 1 equal numbers and one outlier that is spread octaves above them. The outlier lifts the average by only spread / ARRAY\_SIZE, so the outlier's distance from E is:

```math
spread - \frac{spread}{ARRAY\_SIZE} = spread \cdot \frac{ARRAY\_SIZE - 1}{ARRAY\_SIZE}
```

At ARRAY\_SIZE = 4 that is 3/4 of the spread. Case A is the most forgiving shape and case B the least, so real blocks fall between the two.

**Part 3: how much reach we can afford.** The regression's pass mark is 10% error on a product. A product combines two stored numbers, and in the PE their logs are added (Step 4), so their log errors add too. Each is at most step/2, so the product's log error is at most step and its value error at most 2^step - 1:

- scale 5 (step 1/8): 2^(1/8) - 1 = **9.05%**, under 10%
- scale 6 (step 1/4): 2^(1/4) - 1 = **19%**, over

So the scale must be at most 5, which gives step = 1/8 and a reach of 31 x 1/8 = **3.875 octaves** with a 6-bit L.

**Part 4: the maximum spread.** The farthest number has to be within that reach:

| Block shape | Condition | Max spread, 6-bit L | Max spread, 8-bit L (reach 15.875) |
| --- | --- | --- | --- |
| Evenly spread | spread / 2 ≤ 3.875 | 7.75 octaves = 2^7.75 ≈ **215x** | 31.75 octaves ≈ 3.6 x 10^9 |
| One outlier | spread x 3/4 ≤ 3.875 | 5.17 octaves = 2^5.17 ≈ **36x** | 21.2 octaves ≈ 2.4 x 10^6 |

**In plain words:** with the default 6-bit L, the 4 numbers of one block should stay within about 36x (one outlier) to 215x (evenly spread) of each other to keep products under 10%. Each extra bit of L doubles the reach, and so doubles the allowed spread in octaves.

**One correction.** E is rounded to a whole number (2a), which can move it up to 0.5 octave off the true middle. In the worst case that eats 0.5 octave of the reach, so the true limits are slightly tighter than the table.

## Step 3: into the PE

The edge register bank sends one number at a time into the array. E, sign and zero travel on their own wires. L is turned back into a log value, in units of 1/2^FRAC\_WIDTH octave, by shifting it left by the scale (the descaler):

```math
L_{bus} = L \cdot 2^{scale} \qquad\text{so}\qquad \frac{L_{bus}}{2^{FRAC\_WIDTH}} = L \cdot step \approx L_{true}
```

L\_bus goes on a signed bus of MANTISSA\_WIDTH bits, so it must fit:

```math
-2^{MANTISSA\_WIDTH-1} \;\le\; L \cdot 2^{scale} \;\le\; 2^{MANTISSA\_WIDTH-1} - 1
```

**Why this becomes a rule on the scale alone:**

1. The largest L in size is L\_MIN = -2^(BLOCK\_MANTISSA\_WIDTH - 1), i.e. -32. It is one bigger than L\_MAX = +31 (2d), so it is the worst case.
2. Shifted by the scale, it becomes -2^(BLOCK\_MANTISSA\_WIDTH - 1) x 2^scale = -2^(BLOCK\_MANTISSA\_WIDTH - 1 + scale).
3. The bus holds down to -2^(MANTISSA\_WIDTH - 1). So we need 2^(BLOCK\_MANTISSA\_WIDTH - 1 + scale) ≤ 2^(MANTISSA\_WIDTH - 1), i.e. BLOCK\_MANTISSA\_WIDTH - 1 + scale ≤ MANTISSA\_WIDTH - 1.
4. Cancel the -1 on both sides:

```math
scale \;\le\; MANTISSA\_WIDTH - BLOCK\_MANTISSA\_WIDTH
```

**In words:** a BLOCK\_MANTISSA\_WIDTH-bit number shifted left by scale has BLOCK\_MANTISSA\_WIDTH + scale bits, and that has to fit in MANTISSA\_WIDTH bits.

*Check at the default (16-bit bus, 6-bit L):* scale ≤ 10. At scale 10: -32 x 2^10 = -32768, exactly the bottom of a 16-bit signed number, so it just fits. At scale 11: -32 x 2^11 = -65536, which does not fit.

**Example:** the 20 from Step 2 has L = 19 and scale = 5, so L\_bus = 19 x 32 = 608, which is 608 / 256 = 2.375 octaves.

**Default limit:** scale at most 16 - 6 = 10. A larger scale wraps on the bus with no flag. The block's scale field can hold up to 15, so this has to be respected by whoever builds the blocks. In practice Step 2 limits accuracy long before: scale 6 already costs 9% per number.

## Step 4: the multiply, in log form

In one PE a number a from a row block meets a number b from a column block. Their product is formed without a multiplier, only adds:

```math
a \cdot b = (-1)^{sign_a \oplus sign_b} \cdot 2^{\,E_{row} + E_{col}} \cdot 2^{\,L_{row} + L_{col}}
```

Here L\_row and L\_col are the two leftovers in octaves (L\_bus / 2^FRAC\_WIDTH from Step 3).

| Part | What the PE does | Width | Can it overflow? |
| --- | --- | --- | --- |
| sign | sign\_a XOR sign\_b | 1 bit | no |
| zero | zero\_a OR zero\_b (product is 0 if either is) | 1 bit | no |
| exponent | E\_row + E\_col | EXP\_WIDTH + 1 bits | no: the sum of two EXP\_WIDTH numbers always fits one extra bit |
| leftover | L\_row + L\_col | MANTISSA\_WIDTH + 1 bits | no, for the same reason |

**Example:** a = 20 and b = 5, both from blocks with E = 2 (Step 2). The leftovers are 2.375 and 0.375 octaves, so the PE holds exponent 4 and leftover 2.75. The product it represents is 2^(4 + 2.75) = 2^6.75 = 107.6, against the true 100. The +7.6% is exactly the two input rounding errors (3.7% each) combined.

Nothing is lost in this step. All the remaining limits come from turning 2^(L\_row + L\_col) back into a normal number.

## Step 5: back to a normal number (antilog and shift)

After Step 4 the PE holds the product as a log: exponent E\_row + E\_col and leftover L\_row + L\_col. To add products together, it must turn 2^(L\_row + L\_col) back into a normal binary number. This step is where the two product limits come from.

Running example: 20 x 5 from Step 4, leftover **2.75** octaves, exponent 4.

### 5a. Split the leftover into a whole part and a fraction

```math
L_{row} + L_{col} = k + f, \qquad k = \lfloor L_{row} + L_{col} \rfloor, \qquad 0 \le f < 1, \qquad 2^{L_{row}+L_{col}} = 2^{k} \cdot 2^{f}
```

**Why split.** 2^k with a whole number k is just a shift, which is free in hardware. Only 2^f needs real computation, and because f is between 0 and 1, 2^f is always between 1 and 2: a small, fixed range that a small circuit can handle.

**Why rounded down (floor), not to nearest.** Rounding down keeps f positive even for a negative leftover, so 2^f always stays in 1 to 2. For example -1.3 = -2 + 0.7, so 2^(-1.3) = 2^(-2) x 2^(0.7).

*Example:* 2.75 → k = 2, f = 0.75.

### 5b. The fraction: 2^f by CORDIC

The CORDIC unit computes 2^f as e^(f x ln2) and returns it as an integer A with FRAC\_WIDTH fraction bits:

```math
A = \mathrm{round}\big(2^{f} \cdot 2^{FRAC\_WIDTH}\big)
```

**How many bits A needs.** Since 1 ≤ 2^f < 2, multiplying by 2^FRAC\_WIDTH = 256 gives 256 ≤ A < 512. Every such number is 1 followed by 8 more bits in binary (256 = 100000000), so A always has **FRAC\_WIDTH + 1 = 9 bits**:

```math
2^{FRAC\_WIDTH} \le A < 2^{FRAC\_WIDTH+1} \quad\Longrightarrow\quad A \text{ has } FRAC\_WIDTH + 1 \text{ bits}
```

*Example:* 2^0.75 = 1.682, so A ≈ 1.682 x 256 ≈ 431 (binary 110101111, 9 bits).

### 5c. The whole part: shift by k, and the limit for big products

Multiplying A by 2^k is a shift, written into a register P of DATA\_WIDTH bits:

```math
P = A \cdot 2^{k} \;\;(\text{shift left by } k \text{ if } k \ge 0, \text{ right by } |k| \text{ if } k < 0), \qquad \frac{P}{2^{FRAC\_WIDTH}} \approx 2^{\,L_{row}+L_{col}}
```

*Example:* P = 431 shifted left by 2 = 1724. 1724 / 256 = 6.73, and with the exponent: 6.73 x 2^4 = 107.8, the stored version of 20 x 5 = 100.

**How many bits P needs.** Shifting left by k adds k zeros on the right, so P needs (FRAC\_WIDTH + 1) + k bits. The register has DATA\_WIDTH bits, so:

```math
FRAC\_WIDTH + 1 + k \;\le\; DATA\_WIDTH \quad\Longleftrightarrow\quad k \le DATA\_WIDTH - FRAC\_WIDTH - 1
```

k is the leftover rounded down, so k ≤ 23 is the same as the leftover being below 24:

```math
L_{row} + L_{col} \;<\; DATA\_WIDTH - FRAC\_WIDTH \qquad (= 24 \text{ octaves by default})
```

**What happens past it.** The top bits do not fit, are cut off, and the product becomes garbage. There is **no flag** for this.

*Check, test 90:* row 2 of A is (0.00042, 0.00103, 0.0053, 1152). The small numbers pull its E down to -5, so 1152 sits log2(1152) - (-5) = 15.17 octaves above E. Column 3 of B is (1.10, 0.0085, 0.59, 558) with E = 0, so 558 sits 9.12 octaves above E. In the PE they meet: 15.17 + 9.12 = 24.29 octaves, so k = 24, and P would need 9 + 24 = 33 bits in a 32-bit register. The expected 643,203 came out as 0.0032.

**When this happens in general.** One huge number shares a block with tiny ones, so E is low and the huge number's leftover is large. If it meets another large leftover in a PE, the sum passes the limit.

### 5d. The limit for small products

If the leftover is negative, k is negative and A is shifted **right** by abs(k). Each right shift halves the number and drops its lowest bit.

*Example:* with k = -3, P = 431 shifted right by 3 = 53 (exactly 53.9). The 0.9 that fell off is lost.

**When the product vanishes.** A has FRAC\_WIDTH + 1 bits, so after FRAC\_WIDTH + 1 right shifts nothing is left and P = 0:

```math
|k| \ge FRAC\_WIDTH + 1 \quad\Longleftrightarrow\quad L_{row} + L_{col} \;<\; -FRAC\_WIDTH \qquad (= -8 \text{ octaves by default})
```

**Why this is usually harmless.** What falls off is always less than one unit of P. One unit of P is worth 2^(-FRAC\_WIDTH) x 2^(E\_row + E\_col), so each term loses less than:

```math
\text{lost per term} \;<\; 2^{\,E_{row} + E_{col} - FRAC\_WIDTH}
```

That is a tiny fixed amount. It only matters if **every** term of the dot product is that small, and then E was badly chosen for these numbers.

*Check, test 107:* every row and column is (2^-10, 2^-10, 2^10, 2^10), with E = 0. The small terms are 2^-10 x 2^-10 = 2^-20, a leftover of -20 octaves, so they become 0. The big terms are 2^10 x 2^10 = 2^20, a leftover of 20 octaves, inside the limit of 24. The result, 2,097,152, is exact, because the dropped terms are about 10^-12 of it.

### 5e. Small errors in this step

- **ln2 from shifts:** the PE builds ln2 as 1/2 + 1/8 + 1/16 + 1/128 = 0.6953, against the true 0.6931. That is 0.31% too large inside the exponent, which costs at most about 0.2% in value.
- **Last CORDIC iteration:** the 8th iteration's constant cannot be written in the 3-bit shift fields, so that iteration does nothing. It misses a correction of about 0.4% when it would have fired.

Together these stay well under 1% per product.

## Step 6: the sum

Each PE adds ARRAY\_SIZE products, one from each position k of its row and column. The sum can be taken in the linear domain only because of one property:

**Assumption:** all products in one PE share the same E\_row + E\_col, because a row block and a column block each have a single E. So the PE adds the plain P values and keeps E\_row + E\_col once.

**6a. The accumulator.** It is signed with DATA\_WIDTH + 1 bits. Each P is added or subtracted by its sign bit, and skipped if its zero flag is set:

```math
Res\_out = \sum_{\text{terms}} \pm P, \qquad |Res\_out| < 2^{DATA\_WIDTH}
```

**6b. The limit for the sum.** A single product may fit (Step 5c) while the sum of several does not, because adding numbers can make them longer.

1. **How big one product can be.** From 5b and 5c, A < 2^(FRAC\_WIDTH + 1) and P = A shifted left by k, so every product is below 2^(FRAC\_WIDTH + 1 + k).
2. **How big the sum can be.** The worst case is all ARRAY\_SIZE products at that largest size, all with the same sign, so nothing cancels. Their sum is then below ARRAY\_SIZE x 2^(FRAC\_WIDTH + 1 + k).
3. **What the accumulator holds.** It has DATA\_WIDTH + 1 bits: one for the sign and DATA\_WIDTH for the size. So the size must stay below 2^DATA\_WIDTH.
4. **Put them together** and take log2 of both sides:

```math
ARRAY\_SIZE \cdot 2^{\,FRAC\_WIDTH + 1 + k} \le 2^{DATA\_WIDTH} \quad\Longleftrightarrow\quad \log_2(ARRAY\_SIZE) + FRAC\_WIDTH + 1 + k \le DATA\_WIDTH
```

5. **Solve for k** and, as in 5c, turn the bound on k into a bound on the leftover:

```math
k \le DATA\_WIDTH - FRAC\_WIDTH - 1 - \log_2(ARRAY\_SIZE) \quad\Longleftrightarrow\quad L_{row} + L_{col} \;<\; DATA\_WIDTH - FRAC\_WIDTH - \log_2(ARRAY\_SIZE)
```

**Why the extra log2(ARRAY\_SIZE).** Adding N numbers of the same length can need up to log2(N) more bits. For example, 4 numbers of 9 bits each (at most 511) can add up to 2044, which needs 11 bits: 2 more, and log2(4) = 2.

**Default value:** 32 - 8 - 2 = **22 octaves**. No product term may be more than 2^22 (about 4 million) times 2^(E\_row + E\_col). This is 2 octaves stricter than the single-product limit of 24, and like it there is no flag.

**It is a safe bound, not an exact one.** It assumes the worst case of step 2. When terms differ in size or sign, the real sum is smaller, so a block can sometimes go slightly past 22 octaves and still be correct.

**6c. What Res\_out means.** Res\_out counts in units of 1/2^FRAC\_WIDTH, relative to the PE's exponent, so the PE's result is:

```math
C_{ij} = \frac{Res\_out}{2^{FRAC\_WIDTH}} \cdot 2^{\,E_{row} + E_{col}}
```

The PE also outputs E\_row + E\_col as exp\_out (EXP\_WIDTH + 1 bits). This pair is the GOT C matrix in the test report.

## Step 7: the output block

Each column of C (ARRAY\_SIZE results) is packed back into a block of exactly the input format, so it can be fed in again. This repeats Step 2 on the results, with one difference: the log is now computed by hardware, approximately.

**7a. Sign and zero.** sign = 1 if Res\_out < 0; zero = 1 if Res\_out = 0. Both are exact.

**7b. The log of each result.** From Step 6c, log2|C| splits into a part that needs a log circuit and a part that is plain integer arithmetic:

```math
\log_2|C| = \log_2|Res\_out| - FRAC\_WIDTH + (E_{row} + E_{col})
```

The second part is exact integer arithmetic. The first part, log2|Res\_out|, needs a log circuit, and the hardware uses **Mitchell's method**. It is derived below in four parts.

**Part 1: write the number as 2^p x (1 + x).** Let p be the position of the leading 1 of |Res\_out| (bit 0 is the last bit). That leading 1 is worth 2^p, and everything after it is a remainder smaller than 2^p. Pulling out 2^p:

```math
|Res\_out| = 2^{p} + \text{rest} = 2^{p}\Big(1 + \frac{\text{rest}}{2^{p}}\Big) = 2^{p}(1 + x), \qquad 0 \le x < 1
```

Dividing by 2^p just moves the binary point to right after the leading 1, so **x is simply the bits after the leading 1, read as a binary fraction**.

*Example:* 1724 = 11010111100 in binary. The leading 1 is at bit 10, so p = 10. The bits after it are 1010111100, so x = 0.1010111100 in binary = 0.684.

**Part 2: the exact log and Mitchell's shortcut.** Taking log2 of Part 1 gives the exact log:

```math
\log_2|Res\_out| = p + \log_2(1 + x)
```

The integer p is free: it is just the position of the leading 1. The hard part is log2(1 + x). Mitchell's method replaces it with x itself:

```math
\log_2|Res\_out| \;\approx\; p + x
```

In hardware: find the leading 1 (gives p), shift the number so that 1 sits at the top, and take the next FRAC\_WIDTH bits as x. No multiplier, no table. Only the first FRAC\_WIDTH bits of x are kept, which costs at most another 1/256 octave.

*Example:* p + x = 10 + 0.684 = 10.684. The exact log2(1724) is 10.752.

**Part 3: why the shortcut is always a little low.** Compare the two curves for x from 0 to 1:

| x | log2(1 + x), exact | x, Mitchell | gap |
| --- | --- | --- | --- |
| 0 | 0 | 0 | 0 |
| 0.25 | 0.322 | 0.25 | 0.072 |
| 0.443 | 0.529 | 0.443 | 0.086 |
| 0.5 | 0.585 | 0.5 | 0.085 |
| 0.75 | 0.807 | 0.75 | 0.057 |
| 1 | 1 | 1 | 0 |

Both curves start at 0 and end at 1. The exact one bends upward in between, and Mitchell's x is the straight line joining its two ends. A curve that bends this way always lies above the straight line between its ends. So the gap is never negative: **Mitchell never gives a log that is too high.**

**Part 4: the largest gap.** Call the gap g(x) = log2(1 + x) - x. It is 0 at both ends, so its largest value is where its slope is 0:

```math
g'(x) = \frac{1}{(1+x)\ln 2} - 1 = 0 \quad\Longrightarrow\quad x = \frac{1}{\ln 2} - 1 \approx 0.443
```

```math
g(0.443) = \log_2(1.443) - 0.443 = 0.529 - 0.443 \approx 0.0861 \text{ octave}
```

A log error of e octaves is a value error of 2^e - 1 (Step 2e, Part 2). So:

```math
0 \;\le\; \text{log error} \;\le\; 0.0861 \text{ octave} \quad\Longrightarrow\quad \text{value error} \;\le\; 2^{0.0861} - 1 = 6.2\%, \text{ always too low}
```

Powers of two have x = 0, so they come out exact. The worst case is a number whose bits after the leading 1 read as about 0.443.

*Example:* 1724 has x = 0.684, where the gap is 0.068 octave (10.752 - 10.684). That is a 4.8% low value, inside the 6.2% bound.

**7c. E of the output block.** As in Step 2a, E is the rounded average of the logs of the non-zero results. It must fit EXP\_WIDTH bits like the input:

```math
E_{out} = \mathrm{clamp}\big(\mathrm{round}(\text{mean log}_2|C|),\; E_{MIN},\; E_{MAX}\big)
```

**Why a clamp is needed.** Each result's exponent E\_row + E\_col has EXP\_WIDTH + 1 bits (Step 4), and Res\_out can add up to DATA\_WIDTH more octaves. So a result can be far bigger, or smaller, than any E that EXP\_WIDTH bits can store. The output E field is still EXP\_WIDTH bits, the same as the input, so that output blocks can be fed back in.

**What the clamp does.** If the rounded average is above E\_MAX, E is set to E\_MAX and **exp\_overflow** = 1. If it is below E\_MIN, E is set to E\_MIN and **exp\_underflow** = 1. The flags are outputs of the module, not bits inside the block, so the block layout stays identical to the input.

**Why clamp and not just keep the low bits.** Keeping only the low EXP\_WIDTH bits of E wraps it to a completely different value, and the leftovers L then have to make up a huge gap. Clamping keeps E as close as the field allows and raises a flag.

*Check, test 120:* every input is 10^30, so every result is about 4 x 10^60 = 2^201.3 and the ideal E is 201, above E\_MAX = 127.

- **Wrapped** (the old behaviour): the low 8 bits of 201 read as 201 - 256 = -55. L had to carry 201.3 - (-55) = 256.3 octaves, which forced scale 13 (32-octave stairs).
- **Clamped** (now): E = 127, exp\_overflow = 1 on all 4 blocks.

Either way the values are only approximate, because L has to carry the overflow. The difference is that the flag now tells the driver to rescale, instead of a wrong value passing silently.

**7d. L and scale.** Exactly as in Steps 2c and 2d: leftover = log2|C| - E\_out, the smallest scale that fits L\_MIN..L\_MAX, L rounded to the nearest unit. So the output adds the same rounding error as Step 2e, with the step of the output block.

**7e. Total error of one output number,** compared with the ideal result:

```math
\text{error} \approx \underbrace{\text{input rounding (Step 2e, both operands)}}_{\text{and the PE (Step 5e)}} + \underbrace{\text{Mitchell} \le 6.2\%}_{7b} + \underbrace{\text{output rounding}}_{7d}
```

## Where the error comes from

Inside all the limits, the Mitchell log of Step 7b is the largest error source. It is about 7 times the format's own rounding.

**Measured** on the benign tests (1-60, no wide ranges) at the default settings. Each error is shown as a share of the largest value in its column:

| Source | Derived in | Mean | Worst |
| --- | --- | --- | --- |
| PE result: input rounding and CORDIC | Steps 2e, 5e | 0.74% | 5.5% |
| Rounding of an ideal output block | Step 7d | 0.39% | 7.1% |
| Hardware output encoder, mostly Mitchell | Step 7b | 2.60% | 9.0% |

**Each limit checked against the regression** (136 tests):

| Limit | Derived in | Test | What the derivation predicts | What happened |
| --- | --- | --- | --- | --- |
| Spread in a block | Step 2g | spread 100x in block (6.6 octaves) | fits at scale 5 | passes |
| Spread in a block | Step 2g | wide range 1e-2 to 1e2 (13.3 octaves) | needs scale 6-7, 9-19% per number | fails at 6-bit L; all pass at 8-bit L |
| Big products | Steps 5c, 6b | test 90 (leftover sum 24.29) | over the 24-octave limit | 643,203 came out as 0.0032 |
| Small products | Step 5d | test 107 (terms of 2^-20) | dropped, negligible | exact result |
| Output exponent | Step 7c | test 120 (ideal E = 201) | clamped to 127, flagged | clamped and flagged on all 4 blocks |

With BLOCK\_MANTISSA\_WIDTH = 8, EXP\_WIDTH = 12 and DATA\_WIDTH = 48, all 136 tests pass, worst error 6.8%. That configuration widens the spread, exponent and big-product limits together.

The obvious hardware improvement: a small correction to Mitchell (a few adders, or an 8- to 16-entry table on x) would cut its 0.086-octave worst case to about 0.01.

## What the driver should check

Every limit depends only on the blocks the driver builds, so it can be checked in software before a tile is sent.

1. **Exponent (Steps 2a, 7c):** each block's E must lie within E\_MIN..E\_MAX. If not, scale the tile by a power of two and keep the factor.
2. **Spread (Step 2g):** a block that needs scale > 5 will be coarse. Regroup the data so wide-range values do not share a block, or use a wider BLOCK\_MANTISSA\_WIDTH.
3. **PE bus (Step 3):** never send a block with scale > MANTISSA\_WIDTH - BLOCK\_MANTISSA\_WIDTH. This includes output blocks fed back in.
4. **Big products (Steps 5c, 6b):** for each row block and column block pair, the largest leftover of the row plus the largest of the column must stay below DATA\_WIDTH - FRAC\_WIDTH - log2(ARRAY\_SIZE). If not, split or rescale the tile. The hardware has no flag for this, so this check is the only protection.
5. **Output flags (Step 7c):** after each output block, read exp\_overflow and exp\_underflow. If either is 1, rescale and rerun that tile.

**Open hardware item:** an overflow flag on the PE product and accumulator would make check 4 a hardware guarantee instead of a software one.
