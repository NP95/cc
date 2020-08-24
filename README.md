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

# Trace format

Stimulation stimulus can be defined using the following grammar:

```
CPU_ID = [0-9]+

TIME_DELTA = [0-9]+

ADDR = [0-9]+

COMMAND = LD | ST

STATMENT = PATH_STATEMENT | TIME_STATEMENT | COMMAND_STATEMENT

PATH_STATEMENT = 'M' ':' CPU_ID ',' PATH '\n'

TIME_STATEMENT = '+' TIME_DELTA '\n'

COMMAND_STATEMENT = 'C' ':' CPU_ID ',' COMMAND ',' ADDR '\n'

```

Load and store instructions are issued to each CPU instance in the
simulation. At present, there is little ability for the stimulus to
control the ACE commands emitted by the cache controller as this is
itself some function of prior instructions and the implementation. CPU
are identified by a unique ID which is defined by map statements
prepended to the header of the stimulus file. These map statements
form a relation between a unique integer ID, and the CPU path within
the simulation object heirarchy. The time at which stimulus is emitted
by the CPU is approximated by a cursor present which advances in
response to a time statement. The time at which commands issue into
the simulation is only approximate as back-pressure may occur during
the execution of prior commands, which then causes later commands to
be delayed.

There is limited ability to detect malformed trace files, therefore
caution should be applied during their construction.

## Path Statement

Path statements map a simulated CPU instance with a unique integer
ID. By performing the map between ID and path once at the start of the
simulation, the costly instance resolution process, which requires the
path string to be parsed, can be avoided for all subsequent
commands. If the path cannot be found within the simulated design
heirarchy, the model parse process is halted and the simulation
terminated.

The following path statement maps `CPU_ID` 0 to the simulated CPU
instance located at: top.cluster.cpu.

```
M:0,top.cluster.cpu
```

## Time Statement

Time statements advance the time at which commands present in the
stimulus issue. Time statements are relative to prior time statements
and can only advance simulation time. Instructions which are issued
concurrent at the same time to the same CPU are issued sequentially in
the order in which they appear in the simulation file.

The following time statement advances simulation time by 200 time units.

```
+200
```

## Command Statement

Command statements describe a load or store instruction issued by a
simulated CPU instance to its associated L1 cache. Two opcodes are
supported, `LD` which denotes a load instruction and `ST` which
denotes a store instruction. The width of the operation is
indeterminate and assumed to be contained entirely within a single
cache line. There is no ability within the simulation to fetch regions
of memory which straddle multiple lines of memory.

The following command statement, schedules `CPU_ID` 0 to issue a load
to address 0x1000:

```
C:0,LD,0x1000
```
