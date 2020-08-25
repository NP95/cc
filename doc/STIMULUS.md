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
