# PLAN — From Narrative Topics to an Operability Reference

This repo is a systems engineering reference that happens to be executable.
The plan below converts it from “linear curriculum” into a **random-access field manual** without changing the topic arc.

## Step 1 — Make the repo navigable by symptoms (random access)
**Goal:** Let an engineer jump from “what I’m seeing” to “where it’s explained and proven”.

- Add a **Symptom → Topic map** at the top of the README (a troubleshooting matrix / operational failure map).
- Alias topic labels everywhere they appear (README / sidebar / headings) with “problem names”, not just numbers.
  - Example: `Topic 07 — Actions (Starvation failure mode)`

**Why:** The critique’s primary complaint is that the content behaves like a reference, but the docs force sequential reading. The fix is “strictly navigational” via a symptom index :contentReference[oaicite:2]{index=2}.

## Step 2 — Reframe Rust lifecycle rigor as the architectural gold standard
**Goal:** Ensure Rust complexity reads as *architectural strictness*, not “language pain”.

- In lifecycle sections, explicitly frame the Rust managed-node rigor as the **target contract** for C++/Python to emulate.
- Treat shims/adapters as contract enforcers, not patches.

**Why:** The critique warns readers will conclude “Rust is difficult” instead of “this gating is superior architecture” :contentReference[oaicite:3]{index=3}.

## Step 3 — Extend operability beyond bring-up: auditability + long-run health
**Goal:** Make “Day 2” operability part of success, not an afterthought.

- Add verification that:
  - configuration changes are **audited in logs**
  - the system can survive a **simulated long-duration run** without operational failure (e.g., log growth / disk pressure)

**Why:** The critique calls out the “operability gap”: launch success isn’t enough if you can’t reconstruct what happened later :contentReference[oaicite:4]{index=4}.

## Step 4 — Shift Topics 07/08 from “how to break it” to “how to diagnose it”
**Goal:** Teach failure fingerprinting using standard tools, not just failure creation.

- Require diagnostic evidence during starvation:
  - `ros2 doctor`, `ros2 topic hz`, lifecycle timeouts while process still runs
- Add a short “Indicators / Fingerprints” section to Topic 07 theory.

**Why:** The critique explicitly asks for “forensics”: prove it was starvation vs crash vs partition :contentReference[oaicite:5]{index=5}.

<details>
<summary><strong> Critique context</strong></summary>

-
Speaker 1 00:00:00  Hello and welcome to the critique. Today we're dissecting a repository that defines a language neutral behavioural contract for Ros two systems.

Speaker 2 00:00:08  Right. Using that pressure driven curriculum.

Speaker 1 00:00:11  Exactly. It claims to be a systems engineering reference. But it well, it acts more like a linear course.

Speaker 2 00:00:18  A reference that forces me to start at chapter one. That's already a bit of a red flag. Let's get into the architecture.

Speaker 1 00:00:24  Right? So right off the bat, we run into this identity crisis. The material explicitly brands itself as a systems engineering reference. That's a heavy title.

Speaker 2 00:00:34  It implies I can pull this off the shelf when my hair is on fire and find an answer precisely.

Speaker 1 00:00:40  Yet the whole structure is strictly linear. Topic zero zero through topic ten. It's a narrative arc.

Speaker 2 00:00:46  Which is fine for a university course, but you know, it's fatal for a reference manual. Yeah, I mean, think about the user persona here. We aren't just talking about a student learning Ros two for the first time.

Speaker 1 00:00:58  No, the target is a lead engineer or an architect.

Speaker 2 00:01:01  Exactly. If I'm that lead engineer dealing with an executor starvation issue. My threads are locking up. Latency is spiking. I need to jump straight to the solution.

Speaker 1 00:01:10  Which is buried in topic zero eight. In this case.

Speaker 2 00:01:13  And because of that topic zero 0 to 10 labeling, I'm psychologically forced to think, well, I haven't done topic zero zero yet, so I can't look at topic zero eight. It's a wall.

Speaker 1 00:01:24  It creates a prerequisite wall. That's a good way to put it. The friction is that the architecture is random access, but the documentation is sequential.

Speaker 2 00:01:32  It is.

Speaker 1 00:01:32  If I'm debugging a race condition in a compose system, I do not want to wade through a hello world and topic zero zero. The weakness is that conflict between need and format.

Speaker 2 00:01:42  So we need to smash that wall. And the fix isn't to rewrite the code. The code's actually quite good. Oh yeah, the fix is strictly navigational. We need to decouple the systems concepts from the sequential topics.

Speaker 1 00:01:57  How?

Speaker 2 00:01:58  By introducing something like a symptom based index or an operational failure map.

Speaker 1 00:02:04  A symptom based index. Walk me through what that looks like in practice.

Speaker 2 00:02:09  Yeah, you decouple the concepts from the chapter numbers. Right now the Readme lists topic oh seven blocking calls.

Speaker 1 00:02:16  Okay, that's a feature description.

Speaker 2 00:02:18  It describes what the code does. We need a failure description that describes what the user is suffering from. Imagine a troubleshooting matrix at the very top of the readme I see.

Speaker 1 00:02:28  So instead of a table of contents, you have a table of pain points. Exactly. My node stops responding to control C, you click that and it takes you immediately to topics zero zero on signal handling and topic zero eight on executors.

Speaker 2 00:02:41  Or my system works in isolation but fails when I compose it.

Speaker 1 00:02:44  Boom. Link to topic nine and topic ten. You aren't changing the content at all, you are just changing the door the user walks through.

Speaker 2 00:02:51  That completely reframes the entire repo. It respects the user's intelligence. You could even alias the navigation bar itself for sure.

Speaker 1 00:02:59  Right now the sidebar probably just says topic 2007.

Speaker 1 00:03:02  That tells me nothing unless I've memorized the syllabus.

Speaker 2 00:03:05  Right?

Speaker 1 00:03:05  Rename it or alias it. Label it. Topic zero seven the starvation failure mode. Now the user scanning the sidebar sees their problem immediately, not a lesson. Number two support the engineer who comes with a specific problem, not just the student with a free afternoon.

Speaker 2 00:03:20  Absolutely. And it's just text editing, but it transforms the utility of the project.

Speaker 1 00:03:25  Okay, let's pivot to the code itself. There is a fascinating tension in the rust implementation. this project claims to be language neutral, treating C++, Python, and rust as equals. But when you look at the lifecycle implementation and rust.

Speaker 2 00:03:41  With CLS and the rose rust tech shim.

Speaker 1 00:03:44  Yeah, the engineering effort is massive compared to the others.

Speaker 2 00:03:47  It's huge. 3006 reads like a dissertation dissertation. You have this three layer separation, this builder style construction, gated execution. It's complex.

Speaker 1 00:04:00  And my worry is that a C++ or Python developer reading this is just going to glaze over completely.

Speaker 2 00:04:07  They'll think, wow, rust is difficult.

Speaker 2 00:04:09  Glad I don't have to deal with that in C++.

Speaker 1 00:04:11  And that is the trap. The weakness here is this abstraction gap. The material, you know, inadvertently frames this rigorous managed node pattern as a workaround.

Speaker 2 00:04:22  Like it's a patch to make rust behave.

Speaker 1 00:04:25  Yes. And they might miss that this explicit gating is actually a superior architectural pattern, regardless of the language it's buying.

Speaker 2 00:04:33  Architectural safety. In reality, that explicit gating where a node literally cannot exist until it's fully configured is a superior pattern for any language.

Speaker 1 00:04:44  So the C++ developer shouldn't be pitying the rust devs.

Speaker 2 00:04:48  They should be copying them.

Speaker 1 00:04:49  So explain that difference. Why should a C++ developer care about this builder pattern if SLP lifecycle already works?

Speaker 2 00:04:57  Because in C++ using RCL cpp lifecycle, it's still very easy to do what I call start and scream.

Speaker 1 00:05:04  Start and scream.

Speaker 2 00:05:05  You initialize the node and before the constructor is even finished, it's publishing garbage data or subscribing to topics before its internal state is ready. Its implicit and eager. Okay, the rest implementation by necessity forces you to be explicit and gated.

Speaker 2 00:05:21  You can't start the node until the builder is satisfied.

Speaker 1 00:05:23  So the suggestion here is to flip the narrative. Don't apologize for the rust complexity.

Speaker 2 00:05:29  Celebrate it.

Speaker 1 00:05:30  Right. Elevate the managed node pattern from a rust specific detail to a top level architectural mandate for all languages.

Speaker 2 00:05:38  Exactly in the general intent. Our philosophy docs explicitly contrast the implicit, eager construction of standard C++ nodes with the explicit gated one from rust.

Speaker 1 00:05:49  And don't just list the differences.

Speaker 2 00:05:51  No. Judge them, judge them. Yes, tell the C++ users we are using this pattern in rust because the language forces us to. But you should voluntarily do this in C++ because it prevents race conditions during startup.

Speaker 1 00:06:05  So you frame the Rustics shim not as a gap filler, but as a.

Speaker 2 00:06:11  A strictness enforcer.

Speaker 1 00:06:13  I like that phrasing. Voluntary strictness. It changes the vibe from we struggle to make this work too. This is the benchmark for how a system should behave.

Speaker 2 00:06:23  It universalizing the lesson. Suddenly, the rust track isn't just for rust people, it's the gold standard for the architecture.

Speaker 2 00:06:30  Right. And practically show them how to do it in C++. Show them a C++ class that mimics the rust builder pattern. That would really drive the point home.

Speaker 1 00:06:39  Moving on to the pressure aspect of the curriculum, the narrative is all about applying pressure to the system to see if it breaks. The curriculum ends at topic ten. Launch success is defined as deterministic. Bring up the system, turns on nodes, find each other. It shuts down cleanly.

Speaker 2 00:06:59  Which validates day one. Congratulations. You deployed. But systems engineering is about day two.

Speaker 1 00:07:06  What happens on Tuesday? What happens three weeks from now?

Speaker 2 00:07:09  Exactly.

Speaker 1 00:07:10  You're talking about the operability gap. Topic ten proves the system can start, but it doesn't prove it can survive.

Speaker 2 00:07:17  Yeah. The weakness is that the scope ends right there. It misses the critical day to pressure log aggregation metric drift, diagnosing failures that happen after a successful launch.

Speaker 1 00:07:29  Right. The current scope misses that whole feedback loop of long term maintenance.

Speaker 2 00:07:34  Think about log aggregation in topic zero zero.

Speaker 2 00:07:37  We set up logging in topic ten we launch. But do we ever verify that the logging strategy doesn't fill up the disk after 48 hours?

Speaker 1 00:07:45  That's a great point. So we need to extend the pressure narrative. We don't necessarily need a whole new module, but we need to inject long term observability into the success criteria.

Speaker 2 00:07:57  Yeah. The suggestion is to extend the narrative to include a topic 11 long term observability or to integrate health contracts into the existing structure.

Speaker 1 00:08:06  You could even do it within topic ten.

Speaker 2 00:08:09  Oh for sure. Introduce a new constraint. The system must run for a simulated long duration. Maybe you artificially fill a log buffer. If the system crashes because the disk is full, you fail. Topic ten.

Speaker 1 00:08:21  I take it a step further. What about Auditability? In topic zero five, we deal with dynamic parameters. It's one thing to say the parameter updated successfully.

Speaker 2 00:08:32  It's another to prove that the change was recorded.

Speaker 1 00:08:36  Exactly. That is so critical. If I change a gain value on a controller at 2 a.m. and the robot crashes at 4 a.m..

Speaker 2 00:08:45  The only way I know it happened is if that parameter change was audited in the logs. The critique is that topic ten verifies the action, but it fails to verify the artifact.

Speaker 1 00:08:57  So the actionable feedback is to add a verification script. Don't just check rows two param get.

Speaker 2 00:09:03  No Parse the actual log file. Prove to me that a future operator can reconstruct the crime scene, right?

Speaker 1 00:09:10  If you can't reconstruct the state change from the logs. The system isn't operable. It's just functional. And there's a massive difference.

Speaker 2 00:09:19  A huge difference.

Speaker 1 00:09:20  This leads perfectly into our final critique regarding diagnostics. Topic oh seven uses a blocking Fibonacci calculation to starve the executor.

Speaker 2 00:09:30  The classic classroom example.

Speaker 1 00:09:32  It is you write a bad callback. The system hangs. Lesson learned.

Speaker 2 00:09:37  But it's a bit of a Fibonacci strawman, isn't it? The material focuses so heavily on causing the failure.

Speaker 1 00:09:44  Rather than diagnosing it.

Speaker 2 00:09:45  Yeah, as I understand it, the acceptance criteria is just reproducing the failure. Telemetry stops, life cycle hangs. But it doesn't emphasize how an operator using standard Ros tooling would identify acceptor starvation specifically versus, say, a network partition or a crashed process.

Speaker 1 00:10:05  Right? In the real world, you rarely write a blocking loop on purpose.

Speaker 2 00:10:09  You inherit it or you write it by accident. If I see a node goes silent, how do I know it's starvation and not a segfault? The weakness is that the repo teaches you how to break it, but not the forensics to realize why it's broken.

Speaker 1 00:10:23  So the suggestion is to shift the focus of seven and eight from creating a bad node to forensics of a bad node.

Speaker 2 00:10:32  Yes, we need to explicitly require the learner to use diagnostic tools during that starvation period. The acceptance criteria for topic seven is currently the system hangs. That's too low a bar.

Speaker 1 00:10:42  The acceptance criteria should be.

Speaker 2 00:10:44  The system hangs and the student can prove it was starvation using standard tools.

Speaker 1 00:10:49  Fingerprinting the failure I like that. So what concrete examples should they include?

Speaker 2 00:10:55  Require the student to use Ros to doctor. Require them to run Ros two topic h look at the specific behavior.

Speaker 1 00:11:01  And what specifically are they looking for?

Speaker 2 00:11:03  Nuance If the process is still running, you can see it in top or top, but the lifecycle state machine is timing out and the topic rate drops to zero.

Speaker 1 00:11:13  That specific combination of symptoms points to starvation.

Speaker 2 00:11:16  Yes, that is incredibly valuable. You're giving them a heuristic. If you see x, y, and Z. Suspect starvation. Add a specific indicator section to theory zero seven.

Speaker 1 00:11:28  And that in topic zero eight, where they fix it with callback groups.

Speaker 2 00:11:32  Show the difference in the introspection. Run a thread visualizer. Show the single threaded failure right next to the multi-threaded solution. Don't just fix the code, visualize the fix.

Speaker 1 00:11:43  That shifts the pedagogy from coding to systems engineering. You aren't just teaching them to avoid sleep call, you're teaching them to be detectives.

Speaker 2 00:11:53  And that's what this repo is supposed to be about its architecture under pressure. The pressure isn't just on the code, it's on the operator. If the operator can't diagnose the break, the architecture failed.

Speaker 1 00:12:04  So a key theme running through all this feedback is the transition from demonstration to operation.

Speaker 2 00:12:10  I think that's the crux of it. The repository is clearly built by people who understand the deep systems engineering challenges of Ros to the pressure driven curriculum is brilliant, but but the documentation wrapping it is slightly trapped between textbook and reference by breaking that linearity, enforcing the architectural lessons across languages.

Speaker 2 00:12:32  Extending the scope to day two and focusing on forensics, you turn this from a great course into an indispensable tool.

Speaker 1 00:12:39  Okay, so let's recap the main points of the critique and summarize the actionable suggestions. We have four main pillars.

Speaker 2 00:12:47  First, accessibility. We need to transform that topic list into a problem solution matrix. Don't make me guess which topic covers my deadlock.

Speaker 1 00:12:55  Give me a symptom based index so I can jump straight to the fire.

Speaker 2 00:12:59  Exactly. Create that troubleshooting matrix in the Readme. Mapping symptoms like nodes stops responding to specific topics.

Speaker 1 00:13:08  Second, the rust life cycle abstraction gap reframe the rigorous rust life cycle pattern as a strictness benchmark for C++ and Python, not just a language patch, right?

Speaker 2 00:13:21  Don't apologize for the rust rust complexity. Sell it as the architectural goal. Encourage C++ users to adopt that same explicit gating. Voluntarily. Frame the shim as a strictness enforcer that C++ developers should envy.

Speaker 1 00:13:36  Third, the day two operability gap push the boundary of operability beyond launch to include auditability and long term health

Speaker 2 00:13:46  Verify log content.

Speaker 2 00:13:48  Add a constraint in topic ten, where the system must run for a simulated long duration to prove the logging strategy is robust, add that verification script that parses logs to prove configuration changes are audited.

Speaker 1 00:14:01  And finally, the Fibonacci strawman and diagnostic Pedagogy. Teach the diagnosis of starvation, not just the mechanics of it.

Speaker 2 00:14:10  Shift the focus from creating a bad node to forensics. Explicitly require the student to use Rs two doctor or look at Rs two topic HS behavior during starvation.

Speaker 1 00:14:22  And add that indicator section to theory.

Speaker 2 00:14:25  Zero seven yes, if life cycle timeouts occur but the process is running suspect starvation, then show how callback groups in topic zero eight change the introspection output.

Speaker 1 00:14:37  Correctness is easy, operability is earned, and clear documentation is how you keep it. We invite the listener to resubmit this material once these structural changes are addressed.

Speaker 2 00:14:49  Absolutely. I'm particularly interested to see if they can pull off the symptom based index without cluttering the Readme. That's a design challenge in itself, but one worth solving.

Speaker 1 00:15:01  Until next time, keep refining your work.

Speaker 1 00:15:04  This has been the critique.

Speaker 2 00:15:06  Goodbye.

</details>