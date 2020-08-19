# CC Architectural Specification

## Concepts in coherence protocols

Caches are a common micro-architectural idiom used to lower latency,
and hence increase bandwidth, to some addressable memory. Caches
operate on the principal of locality where there is some underlying
assumption that state is accessed in some known and predictable
fashion. Caches improve performance by maintaining the recent
"working/active-set" of state used in memory relatively close to where
it is consumed.

In multiprocessor systems, each CPU (or more generally a cacheable
agent) has its own local cache which is addressable only by itself. By
consequence, state may be duplicate locally within a cache and without
management, such duplicated state could diverge over
time. Multiprocessor systems therefore, regardless of implementation,
require that writes be serialized in some known manner. The outward
manifestation of such writes, known as consistency, is not discussed
in this work.

A simplistic solution is to require that a line may exist in only one
cache at any instant. Although this would solve the coherence problem,
such an approach would be far from optimal, since it a large body of
state used by microprocesors is logically independent and restricted
to a local CPU. Similarly, in the case of immutable data (such as the
text section of dynamic libraries), there would be no ability for
multiple CPU to maintain independent cached copies of the state, even
though it is never (or only rarely) modified over the lifetime of the
system.

Cache Coherence Protocols are specifically designed to perform the
following:

1. Ensure that from some number of coherent agents, at most one agent
   has the ability to perform a write/modification of the line. Line
   state should not be duplicated between agents such that multiple
   copies of the same cache line have different values at any point in
   time.
2. Emphasize duplication of immutable state such that multiple
   processors can have multiple copies of the same line.
3. Emphasize processor-to-processor servicing of requests
   (intervention) such that a processor with the line presently
   installed locally, can itself service requests to another processor
   at a lower overall cost than had the request been serviced through
   the interconnect/overall-system.

Cache coherency is a rich topic full of nuance and complexity and a
thorough discussion herein is outwith the scope of this work. The
interested reader is invited to read [1] for further information.

## Software Architecture

## Protocol Deadlock Avoidance

In computer science, the term deadlock is used to describe the state
where a multi-threaded system halts because of the presence of a
circular dependency between two or more independent entities. Such a
term is typically applied to threads in the software sense, but may
also be applied more generally to any system where there are multiple
interacting agents and where some chain of dependency exists for some
higher-level action.

A central aspect of coherency protocol design is to ensure the absence
of protocol deadlocks, as such event have very serious and
catastrophic impact on the host system. Given the complexity of such
protocols and the huge number of architectural states which the model
can reach, it is infeasible to manually analyze the protocol by hand
and therefore one is forced do so in an automated manner. Within the
context of the current architectural model, the following steps have
been made to mitigate the presence of protocol deadlocks:

1. Virtual Channels

As messages arrive at a coherent agent, they are enqueued to await
processing. As some function of the current architectural state of the
system, a message may or may not issue at any point in time. A message
may, for example, become blocked upon the completion of some other
transaction which is currently inflight within the system. Such a
transaction completes when messages associated with it reach the
coherence agent to advance it to an architectural state which allows
the prior dependent message to resume execution. If the dependent
(blocked) message is queued alongside the messages which can allow for
it to become unblocked, the system deadlocks due to a concept known as
Head Of Line (HOL) blocking.

Head Of Line (HOL) blocking is a common architectural idiom seen in
computer architecture and is typically mitigated by using multiple
independent queues for messages belonging to different agents, or
message beloning to different classes. As each, independent, queue is
arbitrated independently, messages which arrive later in time, can
overtake prior messages which may be blocked.

2. Guaranteed Forward Process for certain Message Classes

Transactional messages which are dependent upon the architectural
state of the system may become blocked awaiting the completion of some
prior action. Other classes of non-transactional messages are ensured
to advance regardless of the architectural state of the system. Such
messages are typically responsible for resource replenishment such as
credits or acknowledgements. These messages typically free-up the
execution of other dependent messages. Other classes of messages which
have the architectural guarantee of forward progress are those which
have guarenteed resources reserved before issue, such as in the case
of snoop responses.

3. Resource reservation

Coherency protocols often exhibit expansionary behavior, where a
single message can cause the creation of some number of child messages
which are then propagated through the system. Consider the case of a
command as it has reached the directory and created some number of
dependent snoop commands. Deadlock can occur in this case if the
command is consumed and necessary resources are exhausted before the
operation has completed. Hardware does not typically have the ability
to restart such a process once it has started, therefore the directory
becomes blocked as it cannot process further awaiting messages.

Resource contention deadlocks are avoided by consuming a message, only
once it has been determined that sufficient resources are present to
allow for it to complete atomically. The operation itself may consume
many cycles to complete, but is architecturall guaranteed not to
require resources which may only become available after competion of
the operation.

[1] Coherence textbook
