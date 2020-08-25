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

The simulator is implement on a basic, event-driven simulation
kernel. This kernel takes some (limited) inspiration from standard
modelling libraries such as SystemC.

### Kernel Architecture

A centralized event-queue maintains the set of outstanding actions to
be performed during simulation execution. The event-queue is
maintained in a min-heap type data structure where actions are
strictly ordered by the time at which they are scheduled to
execute. User-defined actions are derived polymorphically from a
common Action base-class, and they when dequeued an overloaded
'evaluation' method is called to invoke the action defined by the
application.

The simulation kernel supports composition through a SystemC-inspired
object model. Top-level agents are defined in terms of an "Agent"
class, which is itself derived from a generic Module class. An agent
object is free to instantiate child modules to construct a heirarchy.

#### Processes

The most basic form of schedulable element in the simuation kernel, is
the Process. When defining a thread of execution, the user application
derives from the base Process class and overloads the elaboration,
evaluation and finalization methods accordingly. Similar to common
event-driven models of computation, the Process class, implements some
notion of a sensitivity list. This sensitivity list roughtly
encapsulates the conditions or events upon which the Processes
evaluation method may be invoked. Evaluation is carried out
sequentially and is not pre-emptible. Although it can be loosely
classified as a thread of computation, internally it is implemented
simply as a method call and not as a POSIX thread. Unlike SystemC,
which implements the notion of an `SC_THREAD`, the basic Process class
does not support co-routine behavior as this is non-trivial to
implement, and otherwise can be easily approximately by using
additional state to recall when the last evaluation completed, and
where it should be restarted.

#### Events

The Event class is the common primitive around which most scheduling
activities take place. They form the basis from which larger activites
may be composed. Processes register themselves sensitive to an Event,
which causes them to be re-evaluated was the event has been
notified. When notified, the Event construct Action objects which are
then scheduled to execute in the following delta-cycle. This action
then consequently calls the evaluation method of the Process.

Events can be further composed into composite events. In the case of
the EventOr, a parent event can be notified whenever one of some
number of subordinate, child events have been notified. This construct
is fundamentally important in the arbitration routines, where the
arbitration process is initiated in response to the arrival of a
message at one of its requester message queues.

#### MessageQueue

Communication is performed using Queue-type data
structures. Notionally, as the simulator communicates by exchanging
messages between agents, these queues are commonly referred to as
MessageQueues (MQ).

Communication is achieved between two proceses, by first evaluating
the consumer and registering the message arrival event associated with
a MQ. The Producer process, which may in turn be evaluated in response
to an event, or may have been set to re-evaluate at some period
interval, calls the "issue" method on the destination MQ. The Issue
method schedules the enqueue into the MQ to occur after some fixed
period (to model transport delay across the channel). When enqueued,
the message arrival causes the Consumer process to wake, which then
becomes free to remove the message from the head of the queue and
begin processing.

The communication approach differs slightly from that of SystemC which
typically models the notion of a channel as a `sc_signal`. A `sc_signal`
is similar in many respects to the notion of a wire/logic is a
standard HDL simulator and is restricted by the fact that it can
maintain only one value at any point during simulation. SystemC does
indeed have the notion of `sc_fifo` channel types which roughly
approximate the MessageQueue seen here.

### Simulation Architecture

#### Builder

#### Messages/Transactions

Communication within the simulation is carried out by deriving from a
common Message base-class. Message types are indentified by a common
MessageClass definition which allows the common base type to be
statically cast to the derived type at a low cost.

As a single transaction may itself spaw some number of messages, a
overarching Transaction class is used annotate the origin of a Message
within the context of the overall simulation. Each message is
annotated with its corresponding transaction instance which persists
until all messages have been processed and the overall operation
completed.

A secondary, and very convenient aspect of the parent Transaction
class is that each state within Agents can be very easily identified
by the transaction instance pointer, which by definition is guarenteed
to be globally unique during the lifetime of the transaction. The
results in a considerable simplification of agent logic, as state can
be recalled entirely by the transaction instance associated with a
message and forgos the requirement for uniquely assigned Message and
Table ID.


#### Protocol Commands

There are two major and orthogonal aspects to the development of the
coherency simulator: the notion of communication and scheduling of
cooperating agents, and the coherence protocol which governs the
behavior of the system. There is justifiable motivation to demarcate
these two aspects such that they may remain relatively uncoupled,
allowing one to change, but not the other. In addition, there are
reasonable complexity concerns motivating this too, as the design
becomes complicated when attempting to consider both elements
simultaneously.

Decoupling is achieved using an interpreted model of protocol
evaluation. Communicating simulation agents respond to messages as
they arrive at their ingress message queues. The Agent maintains a
data structure which represents the architectural state at some point
in time. A message is arbitrated from the winning ingress queue and
the update packet to the architectural state computed. This update
packet notionally describes the set of actions which are to be applied
to the agent to successfully consume the message. The actions may
either then be applied to the agent's architectural state, or it may
be discarded without having altered the state of the model.

The interpretative model has a number of important advantages. The
agent exposes a subset of its interface to the outside world and
therefore mediates (or restricts) the ability of the protocol logic to
influence external state. Similarly, the agent may expose some number
of Opcodes which can then be used by the protocol to control the
outside world. A second, and key, advantage of the model is the
ability to compute the resources required by a speculative action set
without first having to evaluate it. This is fundamentally important,
as it must be known that the action-set can execute atomically with no
dependency on resources (such as credits) which are not available from
the outset. In the absence of this, the Agent may deadlock awaiting
resources that may never become available.


## Protocol

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

### Virtual Channels

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
messages belonging to different classes. As each, independent, queue
is arbitrated independently, messages which arrive later in time, can
overtake prior messages which may be blocked.

### Guaranteed Forward Process for certain Message Classes

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

### Resource reservation

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
many cycles to complete, but is architecturally guaranteed not to
require resources which may only become available after competion of
the operation.

[1] Coherence textbook
