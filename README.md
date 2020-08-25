# Cache Coherency Performance and Validation Model

## Synopsis

Cache coherency protocols are the architectural mechanisms by which
the Single Writer/Multiple Reader (SWMR) invariant in cached,
multiprocessors systems is managed. Although the performance and
functionaly correctness of such protocols form a central aspect of the
modern-day System-On-Chip (SOC) design, there is often considerable
inability to either model the performance characteristics or the
functional correctness of such protocols in the absence of working
RTL. Often there is insufficient time to understand the minutae of a
design before descisions must be made. More often however, the
majority of teams lack individuals with the cross-disciplined skillset
required by the nature of the activity, including: a detailed
understanding of computer architecture, software design, hardware
design, domain knowledge (coherency) and verification. Often the
decision is made to commit to some incremental version of a design
which has gone before, without truely understand the true performance
and/or functional trade-offs until a large commitment has been made,
which can at time lead to significant problems. Indeed, from the
author's own experience, a very large semiconductor company (beginning
with a Q.) produced an ARM-based SOC which exhitbited a very serious
coherence protocol deadlock which was not detected until the near
complete RTL had reached emulation.

## Discussion

Cache coherency protocols are fundamental to the multiprocessor
system-on-chips. Their implmentation is complicated by the following
aspects:

* The state space of a coherency model is very large and there are
  many distinct scenarios to consider when verifying its
  correctness.
* The context of a coherency model is system-wide, which necessitates
  either functional RTL or behavioral models of a large portion of the
  system. Such models are either large and difficult to simulate, or
  they arriveo only relatively late into the overall development
  process.
* The functional correctness of coherency protocols is incredibly
  important and bugs can cause either data corruption or, in the worst
  case, a complete system hang in the case of a protocol deadlock.
* Coherency protocols play a role in the overall performance of a
  multi-processor system as their behavior directly influences the
  ability of CPU to share state between caches and to source state
  through intervention.
* There are many tuneable characteristics of a coherence sub-system
  that may impact performance in difficult to understand ways. The
  number of commands which may be inflight from a CPU may affect
  performance if too small, whereas it may have a negative power and
  area cost if too large. An inefficient directory cache layout may
  necessitate cause unnecessary snoops, or it may cause useful state
  to be evicted from caches because of recalls.

The availability of a software-based performance and verification
model allows for each of these characteristics to be evaluated in the
context of a simplified, but representative, behavioral
simulation. Issues relating to functional correctness or performance
can be analysed and debugged ahead of the final RTL implementation. In
the presence of completed RTL, complex scenarios can be debugged in
the context of a lightweight and fast simulation environment, than be
debugged quicker than a large waveform dump.


A detailed (work in progress) architectural specification of the
performance model can be found at: [Architectural Spec](./doc/ARCH.md)

### Background

This work was originally undertaken during the Covid-19 pandemic and
is intended to demonstrate the author's competency in their areas of
software modelling of computer architecture. An effort such as this is
a large undertaking and is therefore difficult to complete when
working independently and on a part-time basis. Please consider
therefore that the work contained herein may be incomplete and should
not serve the basis as a production quality verification
model. Instead, please consider this work to be a work-in-progress.

# Usage:

The project is structured as a normal C++17 project with only very
limited dependencies on outside libraries. SystemC has not been used.

To compile:

``` shell
git clone http://www.github.com/stephenry/cc
pushd cc
git submodule update
mkdir build
pushd build
cmake ..
make -j
```

# Unit Tests

For the purpose of development, directed unit tests have been
constructed to verify the functional correctness of certain expected
behaviors of the model. The majority of the emphasis of the work to
date has been carried out using directed tests.

To run, unit tests:

``` shell
ctest .
```

The unit test environment constructs a simulation instance, generates
stimulus, runs the simulation and then checks expected
post-conditions. The test fails if these expected post-conditions have
not been attained. There is limited ability also to perform
self-checking within the simulation, which validates the correctness
of events within the model. There are further plans for randomized
test generation (fuzzers) however the model has not yet reach a point
where this can be carried out.

# Driver:

A relatively simple driver program has been implemented which allows
for the test environment to be configured, and for stimulus to be
defined. A top-level JSON configuration is provided, from which the
top-level simulation environment is defined, constructed and
elaborated. The associated trace-file defines the sequence of
Load/Store commands to be issued by the CPU models in the simulation
and the time at which they are scheduled to occur.

A simple trace driven simulation can be invoked using:

``` shell
./driver/driver ./cfgs/base.trace
```

## Example

For the example [JSON configuration](./cfgs/base.json.in), which is a
very simple single CPU system, following trace causes the zeroth CPU
to emit a `LOAD` instruction at time `200` to address `0x1000`, and a
subsequent `LOAD` to the same line at time `400`.


```
// Set paths
M:0,top.cluster.cpu
// Advance cursor
+200
// Issue Load to 0
C:0,LD,0x1000
// Advance cursor
+200
// Issue Load to 0
C:0,LD,0x1000
```

The first `LOAD` instruction causes a compulsary miss in the CPU's
associated cache, which initiates the fill operation. The line is
eventually installed in the cache in the `Exclusive` state, and the
`LOAD` replayed. Sometime later, the second `LOAD` instruction is
issued, hits in the cache and commits immediately (after some small
cache lookup penalty).

The simulate emits a full trace of all messages communicated during
the execution of the stimulus. Upon completion, gathered statistics
are emitted, below:

```
[410.2:FI;top.statistics (M)]:CPU statistics path top.cluster.cpu: '{transaction_count:2}
[410.2:FI;top.statistics (M)]:L1 Cache statistics top.cluster.l1cache: '{store_hit_n:0, store_miss_n:0, load_hit_n:2, load_miss_n:1}
```

The example trace-file can be readily extended as necessary, but as
the simulator remains a work-in-progress, complicated or non-trivial
sequences are not yet guarenteed to complete successfully.
