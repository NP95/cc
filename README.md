# Cache Coherency Performance and Validation Model

## Synopsis

Cache coherency is the architectural mechanism by which the Single
Writer/Multiple Reader (SWMR) invariant in cached, multiprocessors
systems is managed. Although the performance and correctness of such
protocols form a central aspect of the modern-day System-On-Chip (SOC)
design, there is often some considerable inability to either model the
performance characteristics or functional correctness of such
protocols in the absence of working RTL. The reasons for this are
multitude: often there is insufficient time to understand the minutae
of a design before descisions must be made. More often so, the
majority of teams lack individuals with the cross-disciplined skillset
required by the nature of the activity, including: a detailed
understanding of computer architecture, software design, hardware
design, domain knowledge (coherency) and verification. Often the
decision is made to commit to some incremental version of a design
which has gone before, without truely understand the true performance
and/or functional trade-offs until a large commitment has been made
[1].

The purpose of this work is to demonstrate the author's understanding
and competency in all the prequiste skills required to create a
realistic cache coherence performance/protocol validation model. The
project itself is not intended to be production quality, nor do I
recommend that it be used within a production setting. Instead it
represents a realistic, proof-of-concept model that can be extended
and expanded as necessary.

[1] Indeed, from the author's own experience, a very large
semiconductor company (beginning with a Q.) produced an ARM-based SOC
which exhitbited a very serious coherence protocol deadlock which was
not detected until the near complete RTL had reached emulation.

[Architectural Spec](./doc/ARCH.md)
