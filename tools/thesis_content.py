# -*- coding: utf-8 -*-
"""Thesis draft content. Each chapter is a list of blocks:
   ("h3", text)  section heading      ("h4", text)  subsection heading
   ("p", text)   body paragraph       ("lead", term, text)  bold lead-in
   ("note", text) [ADD ...] placeholder
   ("table", caption, header, rows)   ("fig", path, caption, width_in)
"""
FIG = r"C:\Users\muszy\VRP\figures"
DOEFIG = r"C:\Users\muszy\VRP\data\doe\figures"

TITLE = ("JOINT SCHEDULING AND MULTI-AGENT PATH FINDING FOR AGV "
         "PICKUP-AND-DELIVERY ON GRID NETWORKS")

ABSTRACT = [
    ("p", "Fleets of automated guided vehicles (AGVs) move material between "
     "stations in warehouses and factories. Coordinating them requires two "
     "coupled decisions: which robot serves which job, and in what order "
     "(scheduling), and which collision-free route each robot drives "
     "(multi-agent path finding, MAPF). This thesis builds a physical "
     "six-robot testbed to study these decisions under real execution, and "
     "uses it to compare four traffic-planning methods on a resource-"
     "constrained pickup-and-delivery workload."),
    ("p", "The testbed is a 97 in square table carrying an 8 x 8 lattice "
     "with 49 workstation bays and a seven-slot depot. Six Arduino Alvik "
     "robots are localized by an overhead camera reading AprilTag fiducials "
     "(52 frames per second, 0.26 in mean position error), and are driven by "
     "a centralized ROS 2 supervisor over micro-ROS. Allocation is solved by "
     "a biased random-key genetic algorithm; routes are then planned by "
     "uncoordinated A*, delay insertion (WAIT), prioritized planning (PP) or "
     "conflict-based search (CBS)."),
    ("p", "A blocked 4 x 2 factorial experiment (40 runs, five rounds) found "
     "that fleet size dominates makespan (p < 0.001). With six robots the "
     "method mattered (p = 0.001): WAIT was the slowest, and PP finished "
     "fastest by shortening the critical robot's route at the expense of "
     "robots with slack. With four robots no method differed on average. "
     "Every method ran 16-32 s longer than planned, and execution faults "
     "produced run-to-run variation as large as the differences between "
     "methods. The joint planner never outperformed the sequential one, "
     "which motivates the follow-up experiment on a more congested workload."),
    ("note", "Rewrite the abstract last, once Chapter 5 includes the joint-"
     "versus-sequential experiment. Graduate School limit: confirm the word "
     "limit (commonly 350 words)."),
]

ACK = [
    ("p", "I thank my major professor, Dr. Manbir Sodhi, for his guidance "
     "throughout this work. The vehicle-routing solver used in this thesis "
     "originates in the work of Harish Saseendran, Dr. Sodhi and Romesh "
     "Prasad on the vehicle routing problem with resource-constrained pickup "
     "and delivery; this thesis builds directly on it."),
    ("note", "Add committee members, lab colleagues, funding sources and "
     "personal acknowledgments."),
]

PREFACE = [
    ("note", "The Preface is required only for Manuscript Format. This "
     "thesis is written in standard (monograph) format, so delete this page "
     "unless the advisor asks for one."),
]

# --------------------------------------------------------------------------
CH1 = [
    ("h3", "1.1 Background and Motivation"),
    ("p", "Automated warehouse and manufacturing systems increasingly depend "
     "on fleets of automated guided vehicles (AGVs) to transport material "
     "between pickup and delivery stations. Coordinating such a fleet "
     "requires solving two coupled subproblems: scheduling, which decides "
     "which robot handles which job and in what order, and multi-agent path "
     "finding (MAPF), which computes collision-free routes through shared "
     "physical infrastructure. The dominant practice in industry and in much "
     "of the academic literature is to solve these subproblems sequentially "
     "or in isolation (Wurman et al., 2008; Yang et al., 2023; Zhang et al., "
     "2023). A growing body of recent work argues that treating scheduling "
     "and routing as separate layers is suboptimal: scheduling decisions "
     "that ignore routing create congestion and deadlocks, while routing "
     "decisions that ignore job sequencing fail to minimize the overall "
     "makespan (Chen et al., 2021; Ding et al., 2025; Zhang et al., 2023)."),
    ("p", "Most of this evidence comes from simulation. Physical execution "
     "adds effects that simulation usually omits: communication delay, "
     "localization noise, imperfect stopping and turning, and queueing at "
     "shared resources such as the depot. Whether planning-level advantages "
     "survive these effects is an open, practical question for anyone "
     "deploying a small AGV fleet."),
    ("h3", "1.2 Statement of the Problem"),
    ("p", "This thesis studies resource-constrained pickup and delivery on a "
     "fixed-topology grid network. Robots leave a shared depot, drop jobs at "
     "workstations where processing begins, continue to other work while "
     "processing runs, return for a second visit to pick each job up once "
     "processing is complete, and finish at the depot. The objective is to "
     "minimize makespan, the time from the release of the first robot until "
     "the last robot is parked at the depot. Feasible plans must respect "
     "job precedence, robot capacity, and node and edge conflicts on the "
     "grid."),
    ("h3", "1.3 Research Questions"),
    ("lead", "Primary question.", "Does jointly optimizing scheduling and "
     "routing reduce makespan compared with sequential optimization in a "
     "constrained multi-AGV pickup-and-delivery system on a fixed-topology "
     "network?"),
    ("lead", "Secondary question.", "How well do planning-level performance "
     "differences transfer to physical execution, where communication "
     "delay, sensor noise and queueing at the shared depot are present?"),
    ("lead", "Supporting question (Experiment 1).", "With the allocation "
     "held fixed, does the choice of traffic-planning (MAPF) method change "
     "makespan, prediction error, waiting and safety interventions, and "
     "does any difference depend on fleet size?"),
    ("note", "Confirm with the advisor that the supporting question is "
     "framed as Experiment 1, and that the joint-versus-sequential "
     "experiment (Section 4.6) answers the primary question."),
    ("h3", "1.4 Significance of the Study"),
    ("p", "A foundational assumption in classical vehicle routing is that "
     "scheduling and routing can be decoupled: assign jobs first, then plan "
     "paths. The assumption holds where paths are short, congestion is rare "
     "and travel times are deterministic. In dense multi-robot environments "
     "on fixed-topology networks it breaks down: robots share intersections, "
     "bottlenecks form, and a schedule that is optimal as an assignment may "
     "produce large delays once routing conflicts are resolved."),
    ("p", "The question is directly relevant to Manufacturing Execution "
     "System (MES) design. An MES dispatches work orders to physical "
     "resources in real time; in an AGV-based factory that dispatch function "
     "must coordinate assignment, sequencing and routing together "
     "(Saseendran et al., 2026)."),
    ("h3", "1.5 Contributions"),
    ("lead", "1. An open, documented physical testbed.", "A six-robot AGV "
     "testbed (table, overhead AprilTag localization, firmware, ROS 2 "
     "supervisor, and metrics database) for reproducible multi-robot "
     "pickup-and-delivery research. Chapter 3."),
    ("lead", "2. A problem formulation.", "A statement of the joint "
     "scheduling and routing problem for fixed-topology AGV networks with "
     "resource-constrained pickup and delivery. Chapter 4."),
    ("lead", "3. A controlled hardware comparison of MAPF methods.", "A "
     "blocked factorial experiment comparing four traffic-planning methods "
     "at two fleet sizes on physical robots. Chapters 4 and 5."),
    ("lead", "4. Execution-reliability findings.", "Measured failure modes "
     "of multi-robot execution, such as dropped commands, transport stalls, "
     "rotation geometry and deadlock detection, with their fixes. Chapter 5."),
    ("lead", "5. Joint versus sequential planning.", "An empirical test of "
     "the primary research question."),
    ("note", "Contribution 5 depends on the follow-up experiment. Update "
     "this list once its scope (ALNS-based, more workstations) is fixed."),
    ("h3", "1.6 Organization of the Thesis"),
    ("p", "Chapter 2 reviews the literature on pickup and delivery, MAPF, "
     "integrated planning and physical multi-robot testbeds. Chapter 3 "
     "describes the testbed and system, from the construction of the table "
     "through the localization system and robot hardware to the software "
     "architecture. Chapter 4 formulates the problem and describes the "
     "planning methods and experimental designs. Chapter 5 presents the "
     "findings, and Chapter 6 concludes and outlines future work."),
]

# --------------------------------------------------------------------------
CH2 = [
    ("h3", "2.1 Pickup and Delivery Problems"),
    ("p", "The pickup and delivery problem (PDP) has a well-established "
     "foundation in operations research (Berbeglia et al., 2007; Parragh et "
     "al., 2008). Classical formulations treat travel times as deterministic "
     "parameters and do not model multi-agent collisions. Saseendran et al. "
     "(2026) define the vehicle routing problem with resource-constrained "
     "pickup and delivery (VRP-RPD), in which a job is dropped at a station, "
     "processed for a fixed duration while the vehicle continues elsewhere, "
     "and then picked up by a second visit. They solve it with a biased "
     "random-key genetic algorithm (BRKGA). That solver is the allocation "
     "stage of this thesis."),
    ("h3", "2.2 Multi-Agent Path Finding"),
    ("p", "MAPF computes collision-free paths for a set of agents on a "
     "graph. Stern et al. (2021) give standard definitions of node, edge "
     "and swap conflicts and the common objectives. Conflict-based search "
     "(CBS) is optimal: it plans each agent alone and branches on conflicts "
     "by constraining one agent or the other (Sharon et al., 2015). "
     "Prioritized planning plans agents one at a time against a reservation "
     "table of agents already planned (Erdmann and Lozano-Perez, 1987; "
     "Silver, 2005); it is fast but incomplete. Lifelong MAPF extends the "
     "problem to agents that keep receiving tasks (Ma et al., 2017; Li et "
     "al., 2021), which is the closest MAPF setting to pickup and delivery."),
    ("note", "Add delay-insertion / wait-based conflict repair references, "
     "and robust or execution-aware MAPF (for example, action dependency "
     "graphs), since Experiment 1 kept timed waits and considered event-"
     "based execution."),
    ("h3", "2.3 Integrated Task Assignment and Path Planning"),
    ("p", "Zhang et al. (2023) showed that integrating task scheduling with "
     "MAPF, rather than solving them in sequence, reduces congestion and "
     "deadlocks in dense robot environments. Ding et al. (2025) proposed a "
     "bi-level program, with task sequencing at the upper level and "
     "collision-free path finding at the lower level, that captures the "
     "coupling while remaining tractable. Chen et al. (2021) developed an "
     "integrated framework for capacitated multi-agent pickup and delivery "
     "covering assignment, sequencing and path planning, and showed that "
     "coordinating all decision layers outperforms isolated optimization."),
    ("h3", "2.4 Metaheuristics: BRKGA and Large Neighborhood Search"),
    ("p", "BRKGA encodes a solution as a vector of random keys and relies on "
     "a problem-specific decoder to turn keys into a solution (Goncalves and "
     "Resende, 2011). Large neighborhood search repeatedly destroys and "
     "repairs part of a solution; its adaptive form (ALNS) was introduced "
     "for pickup and delivery with time windows (Ropke and Pisinger, 2006). "
     "In MAPF, LNS is the basis of strong anytime solvers (Huang et al., "
     "2022; Li et al., 2022; Phan et al., 2024)."),
    ("h3", "2.5 Industrial Multi-Robot Systems"),
    ("p", "The best-known physical multi-robot warehouse system is the Kiva "
     "system (Wurman et al., 2008), in which robots navigate with largely "
     "decentralized, reactive collision avoidance rather than globally "
     "coordinated joint optimization. It scales to thousands of robots, "
     "and serves as an industrial baseline for the question of how much "
     "centralized joint optimization improves makespan on a structured "
     "network."),
    ("h3", "2.6 Physical Testbeds and Fiducial Localization"),
    ("p", "AprilTag fiducials give robust, low-cost six-degree-of-freedom "
     "marker detection from a single camera (Olson, 2011; Wang and Olson, "
     "2016) and are widely used for ground truth in multi-robot "
     "laboratories. The testbed in this thesis uses them as its only source "
     "of absolute position."),
    ("note", "Add comparable physical multi-robot testbeds (for example "
     "the Robotarium and Duckietown) and position this testbed against "
     "them: scale, cost, localization method, centralized control."),
    ("h3", "2.7 Research Gap"),
    ("p", "Joint scheduling and routing has been studied mostly in "
     "simulation, and MAPF methods are rarely compared on physical robots "
     "under controlled, replicated conditions. This thesis contributes a "
     "replicated hardware comparison and a testbed on which the joint-"
     "versus-sequential question can be tested under real execution."),
]

# --------------------------------------------------------------------------
CH3 = [
    ("p", "This chapter describes the testbed as built: the table and its "
     "grid, the localization system, the robots and their firmware, and the "
     "software architecture that connects them. Every measured value in "
     "this chapter was recorded on the testbed itself; the full list of "
     "constants is in Appendix A."),
    ("h3", "3.1 System Overview"),
    ("p", "The system has four parts. A planning layer solves the "
     "allocation and traffic-planning problems and freezes the result as a "
     "plan file. A fleet supervisor executes the plan, holding reservations, "
     "detecting conflicts and deadlocks, and driving one navigator per "
     "robot. An overhead camera reads an AprilTag on every robot and "
     "publishes its position and heading. A metrics backend records every "
     "event, pose sample and run video for analysis. The robots carry no "
     "map: they execute one motion command at a time and report their "
     "status."),
    ("fig", FIG + r"\fig_arch.png", "Runtime architecture of the testbed: "
     "the Windows laptop (camera bridge, dashboard, metrics), WSL2 "
     "(AprilTag localization), the Linux laptop (micro-ROS agent, rosbridge, "
     "fleet supervisor and navigators) and the robots.", 6.0),
    ("note", "Update Figure fig_arch: it still shows camera_correction_v2 "
     "firmware and the planning layer as 'not wired'. Planning is now "
     "integrated (traffic planners and frozen DOE plans)."),

    ("h3", "3.2 Table Construction"),
    ("h4", "3.2.1 Table Structure and Surface"),
    ("p", "The testbed is a square table approximately 97 in on a side, "
     "with a matte white surface. It is built from two panels joined along "
     "a north-south seam at the centre of the table (visible in Figure 3.2). "
     "Total material cost was under $250."),
    ("note", "Add construction details: panel material and thickness, "
     "frame and leg construction, table height, how the panels are joined "
     "and levelled, surface paint or laminate, construction time, and a "
     "photograph of the frame. Add a dimensioned drawing (plan view)."),
    ("fig", FIG + r"\testbed_empty.jpg", "The empty testbed seen from the "
     "overhead camera: black tape lattice, workstation spurs, red lattice-"
     "node markers and yellow workstation markers.", 5.5),
    ("h4", "3.2.2 Grid Layout"),
    ("p", "Black tape encodes an 8 x 8 lattice of 64 nodes with a uniform "
     "10.00 in pitch. Node 1 sits at (13.5, 16.75) in from the table's "
     "south-west corner, and nodes are numbered row-major from the south-"
     "west. A junction node, node 0, hangs 13.5 in south of node 1 and is "
     "the single connection between the depot and the grid. Lattice edges "
     "are two-way; direction and occupancy are enforced in software rather "
     "than by the layout."),
    ("note", "Add the tape specification (brand, width), how the lattice "
     "was laid out and squared (for example a chalk-line method), and the "
     "measured lattice error."),
    ("h4", "3.2.3 Workstation Bays"),
    ("p", "Each of the 49 cells enclosed by the lattice contains a "
     "workstation bay, drawn as a tape spur that hangs from the cell's north "
     "edge (Figure 3.2). Workstations are numbered 65 to 113. A robot enters "
     "a bay from an entry node on the lattice edge midway between two "
     "lattice nodes (entry nodes 114 to 162, each offset by 49 from its "
     "workstation), turns south, drives to the workstation and turns to the "
     "180 degree service heading. All 49 workstation positions were "
     "measured with parked robots, each value the median of 60 AprilTag "
     "samples (sample spread 0.01 in)."),
    ("h4", "3.2.4 Depot"),
    ("p", "The depot is a single lane along the south edge of the table "
     "with seven parking slots, D1 to D7, each reached from its own entry "
     "node, DE1 to DE7. Robots park facing south. The measured slot pitch is "
     "6.30-6.50 in. Every robot enters and leaves the grid through node 0 "
     "and node 1, so the depot lane is the testbed's only structural "
     "bottleneck. Robots leave the depot by a cascade launch: all move to "
     "their entry nodes and turn west at once, then enter the lane one at a "
     "time (Section 5.1)."),
    ("note", "Add a depot drawing with slot and entry positions, and how "
     "the slot positions were placed (meter-stick placement, 2026-10-22)."),
    ("h4", "3.2.5 Markers"),
    ("p", "Coloured vinyl markers identify node roles: red at lattice "
     "nodes, yellow at workstations and blue at the depot. They were the "
     "localization reference for the original line-following firmware, "
     "which read them with the robot's colour sensor. In the current "
     "camera-guided system they are visual references for the operator and "
     "a fallback for the colour-guided mode."),
    ("note", "Confirm which markers are still on the table and whether "
     "the colour-guided mode is still supported; otherwise move this to "
     "Section 3.6 (system evolution)."),
    ("h4", "3.2.6 Coordinate System"),
    ("p", "The world frame has its origin at the table corner beside "
     "reference tag 20 (south-west), +x toward tag 21 (east) and +y toward "
     "tag 22 (north), in inches. All 177 named locations (64 lattice nodes, "
     "node 0, 49 workstations, 49 entries, 7 depot slots and 7 depot "
     "entries) are stored in a single coordinates file read by both the "
     "localizer and the navigators, so the two programs cannot disagree "
     "about where a node is."),
    ("h4", "3.2.7 Geometric Constraints"),
    ("p", "The robot's top face is 4.0 x 4.0 in, so a rotating robot "
     "sweeps a circle of 5.625 in diameter. Table 3.1 compares this with the "
     "grid's spacings. A robot at a bay entry cannot rotate without "
     "overlapping a robot on either adjacent lattice node or on its own "
     "workstation. This is a property of the table, not a threshold to "
     "tune, and it shapes the supervisor's rotation rules (Section 3.5.4)."),
    ("table", "Rotation clearance between node classes (robot rotation "
     "diameter 5.625 in).", ["Node pair", "Distance (in)", "Clearance (in)"],
     [["Bay entry - adjacent lattice node", "5.00", "-0.62"],
      ["Bay entry - its workstation", "4.99", "-0.63"],
      ["Lattice - lattice", "10.00", "+4.38"],
      ["Workstation - lattice", "6.92-7.21", "+1.30"],
      ["Depot slot - depot slot", "6.30-6.50", "+0.68 to +0.88"]]),

    ("h3", "3.3 Localization: Overhead Camera and AprilTags"),
    ("h4", "3.3.1 Camera"),
    ("p", "A NexiGo N980P webcam is mounted above the centre of the table, "
     "looking straight down, and streams 1920 x 1080 MJPEG at 60 frames per "
     "second requested. A second overhead camera on a separate computer "
     "records each run for review; it does not take part in localization."),
    ("note", "Add the camera mount (height above the table, bracket, "
     "lighting), the lens field of view, and a photograph."),
    ("h4", "3.3.2 AprilTag Fiducials"),
    ("p", "The system uses the tag36h11 AprilTag family (Olson, 2011; Wang "
     "and Olson, 2016). Four reference tags, IDs 20 to 23, mark the table "
     "corners (20 south-west, 21 south-east, 22 north-west, 23 north-east). "
     "Their black squares measure 3.835 in. The centre of tag 20 is 2.75 in "
     "from both table edges, and the centres of tags 20 and 23 are 91.5 in "
     "apart in both x and y. Each robot carries one tag on its top face "
     "(IDs 1 to 6 for Alvik1 to Alvik6). The reference tags are raised to "
     "the height of the robot tags, so that both lie in one plane and a "
     "single homography maps image to table without parallax error."),
    ("note", "Add: robot-tag size and mounting (photo), reference-tag "
     "riser construction, and the measured positions of tags 21 and 22 (the "
     "software still assumes the same 2.75 in inset)."),
    ("h4", "3.3.3 Pose Estimation"),
    ("p", "Each frame is undistorted and searched for tags. The corners of "
     "every visible reference tag fit an image-to-table homography by a "
     "normalized direct linear transform; with three or more reference "
     "tags, per-tag rotation is estimated freely. Each robot tag's centre "
     "and corner geometry then give the robot's position (x, y) and heading "
     "in the world frame. A pose-jump filter rejects physically implausible "
     "samples, and a light yaw filter smooths heading."),
    ("note", "Add the equations: homography, tag-centre projection, yaw "
     "from the tag's corner vector, and the filter parameters."),
    ("h4", "3.3.4 Vision Pipeline"),
    ("p", "The camera is captured natively on the Windows computer "
     "(camera_bridge_windows.py) and streamed as JPEG frames over a local "
     "TCP socket to the localizer (apriltag_localize.py), which runs under "
     "WSL2 on the same computer. The localizer publishes one pose topic per "
     "robot (/AlvikN_vision_pose, JSON with position, heading, grid "
     "coordinates and capture time) into the ROS 2 graph through a "
     "rosbridge websocket. Passing the USB camera directly into WSL2 was "
     "tested and rejected: it reduced throughput from about 40 Hz to 9 Hz."),
    ("h4", "3.3.5 Performance"),
    ("p", "The CPU pipeline runs at 52.0 frames per second. Robot tags are "
     "detected in 98.8-99.7 percent of frames and the reference tags in 100 "
     "percent. Position accuracy, checked against six surveyed depot "
     "markers, is 0.26 in mean and 0.38 in worst. A GPU AprilTag pipeline "
     "(NVIDIA Isaac ROS) was evaluated and declined because it won no metric "
     "(Table 3.2)."),
    ("table", "CPU versus GPU AprilTag detection.", ["Metric", "CPU",
     "GPU"], [["Throughput (frames/s)", "52.0", "45"],
              ["Detection, Alvik1-5", "98.8-99.4%", "100%"],
              ["Detection, Alvik6", "99.72%", "92.8%"],
              ["Detection, corner reference tag", "100%", "1.6%"]]),

    ("h3", "3.4 Robot Hardware and Firmware"),
    ("h4", "3.4.1 The Arduino Alvik"),
    ("p", "The fleet is six Arduino Alvik robots, Alvik1 to Alvik6. Each "
     "Alvik pairs an ESP32-S3 main processor, which runs the user sketch and "
     "the WiFi link, with an STM32 carrier board that controls the two "
     "drive motors through wheel encoders and reads the inertial "
     "measurement unit (IMU), line sensors and colour sensor. Its top face "
     "is 4.0 x 4.0 in. Each robot identifies itself by its WiFi MAC "
     "address, so all six run identical firmware."),
    ("note", "Add a hardware specification table: dimensions, mass, wheel "
     "diameter, motors and encoders, IMU, sensors, battery capacity and "
     "chemistry, top speed, and the carrier firmware version. Add a "
     "labelled photograph."),
    ("h4", "3.4.2 Firmware"),
    ("p", "The firmware is a reactive executor: it accepts one command at a "
     "time and reports when it is done, with no knowledge of the grid, a "
     "route or a workstation. It exposes four topics per robot: a command "
     "topic, a status topic, a pose topic (wheel odometry and battery, every "
     "50 ms while driving) and a streamed wheel-command topic. Table 3.3 "
     "lists the commands used by the current system."),
    ("table", "Firmware commands used by the camera-guided system.",
     ["Command", "Function"],
     [["WHEEL_FOLLOW_MODE", "Accept streamed left/right wheel speeds from "
       "the navigator (50 Hz). A watchdog brakes the robot if no setpoint "
       "arrives for 1 s."],
      ["ROTATE_REL <deg>", "Closed-loop in-place rotation, completed on an "
       "IMU-confirmed heading; one local retry if no motion is detected."],
      ["STOP", "Brake and return to idle."],
      ["RESET_POSE", "Zero the odometry origin."],
      ["DWELL <ms>", "Wait in place for the given time."],
      ["GET_STATUS", "Report busy or idle state."]]),
    ("note", "Add the firmware version history (line-following colour "
     "firmware, then camera-correction v1-v3) and the reasons for each "
     "change."),

    ("h3", "3.5 Software Architecture"),
    ("h4", "3.5.1 Computers and Roles"),
    ("p", "The runtime uses two laptops on a dedicated WiFi network (Table "
     "3.4). The micro-ROS agent must share a host with the supervisor: the "
     "agent republishes each robot's pose and status as DDS topics, and "
     "splitting the two across a virtualization boundary produced "
     "subscriptions that never matched their publishers. Moving the agent "
     "back to the Linux laptop also removed a class of fleet-wide command "
     "stalls (Section 5.1)."),
    ("table", "Runtime components, hosts and transports.", ["Component",
     "Host", "Transport"],
     [["micro-ROS agent", "Linux laptop", "XRCE-DDS over UDP to the robots"],
      ["rosbridge", "Linux laptop", "WebSocket"],
      ["Fleet supervisor and six navigators", "Linux laptop",
       "rclpy, in-process"],
      ["Camera bridge", "Windows laptop", "TCP, JPEG frames"],
      ["AprilTag localization", "Windows laptop (WSL2)",
       "roslibpy to rosbridge"],
      ["Dashboard, metrics server, recorder", "Windows laptop",
       "Flask, SQLite, HTTP"],
      ["Second overhead camera recorder", "Separate Linux laptop", "HTTP"]]),
    ("h4", "3.5.2 Communication: ROS 2 and micro-ROS"),
    ("p", "The robots talk to the system through micro-ROS (XRCE-DDS over "
     "UDP and WiFi) and appear in the ROS 2 Jazzy graph as ordinary topics "
     "(Macenski et al., 2022). Commands are sent best-effort and status "
     "returns reliably. Because a best-effort command can be lost silently, "
     "every command must be acknowledged by a BUSY status from the robot, "
     "and is re-sent if the acknowledgment does not arrive."),
    ("h4", "3.5.3 Planning Layer"),
    ("p", "Planning runs in a browser dashboard served by the metrics "
     "server. The VRP-RPD solver (BRKGA, Saseendran et al., 2026) assigns "
     "and orders jobs; one of four traffic planners then turns each robot's "
     "visit sequence into a timed path on the grid: uncoordinated A*, WAIT, "
     "PP or CBS (Section 4.3). All planners share one timing model and one "
     "set of conflict rules. A plan is saved as a JSON file and can be "
     "frozen, so that repeated runs drive identical routes. 'Send mission' "
     "publishes the plan to the supervisor."),
    ("h4", "3.5.4 Fleet Supervisor"),
    ("p", "The supervisor (fleetSupervisor.py) executes the plan. Its "
     "dispatch advisor holds a reservation for every node a robot will "
     "occupy, simulates the remaining plan to detect conflicts, and inserts "
     "waits where needed. A wait-for graph detects deadlocks; resolution "
     "moves exactly one robot, which is sufficient because a wait-for cycle "
     "is a simple cycle. A collision guard compares every pair of robots "
     "using their measured footprints (4.00 in square-on, 5.66 in at 45 "
     "degrees) and stops movers on a predicted contact. Two rotation gates, "
     "one on reservations and one on measured camera clearance, prevent a "
     "robot from turning into a neighbour (Table 3.1)."),
    ("note", "Add a state diagram of a robot's life cycle (launch, travel, "
     "service, return, park) and a sequence diagram of one leg (supervisor "
     "-> navigator -> firmware -> status)."),
    ("h4", "3.5.5 Navigation Controller"),
    ("p", "Each robot has a navigator (camera_grid_navigate.py) that drives "
     "one leg at a time. In the Camera + Encoder mode used for all "
     "experiments, the navigator streams wheel speeds at 50 Hz while "
     "steering toward the target node using the camera pose, fused with the "
     "robot's wheel odometry. It brakes at a per-robot brake lead and "
     "applies a per-robot steering trim, both derived from 3,019 recorded "
     "legs. Turns use ROTATE_REL followed by a settle step that holds the "
     "final heading within 2 degrees at workstations and depot slots. A stop "
     "is verified by both the camera and the encoders. If the camera feed "
     "drops, the navigator finishes the current leg on odometry and holds "
     "until the camera returns."),
    ("table", "Per-robot drive calibration.", ["Robot", "Brake lead (in)",
     "Steering trim (RPM)"],
     [["Alvik1", "1.24", "-2.87"], ["Alvik2", "1.23", "-1.66"],
      ["Alvik3", "1.67", "-4.87"], ["Alvik4", "1.24", "-0.99"],
      ["Alvik5", "1.43", "-2.76"], ["Alvik6", "1.42", "-0.19"]]),
    ("h4", "3.5.6 Metrics, Logging and Video"),
    ("p", "A Flask server stores every run in a SQLite database: run "
     "metadata and the frozen plan, robot status changes, commands, pose "
     "and battery samples, conflicts and job events. Metric values are "
     "computed from versioned definitions, so a metric can be recomputed "
     "and its definition audited. Both overhead cameras record each run "
     "automatically, and the two videos are linked to the run record."),
    ("h3", "3.6 Evolution of the System"),
    ("p", "The testbed began as a line-following system: robots followed "
     "the tape and stopped on coloured markers read by the colour sensor, "
     "with odometry as the only position estimate. Overhead AprilTag "
     "localization was added for monitoring and then became the control "
     "input, first assisting and then replacing line following."),
    ("note", "Expand: a dated timeline of the main changes (line "
     "following -> camera-assisted driving -> Camera + Encoder; color "
     "firmware -> camera-correction firmware; single machine -> split "
     "hosts), each with the measured reason."),
]

# --------------------------------------------------------------------------
CH4 = [
    ("h3", "4.1 Operational Definitions"),
    ("p", "Scheduling is the assignment of jobs to robots together with the "
     "sequencing of jobs within each robot's set. Routing is the generation "
     "of node-by-node paths through the grid for each robot, given an "
     "assignment and sequence. Dispatching is the set of real-time "
     "decisions made by the supervisor, including depot release and any "
     "holds. Multi-agent path finding (MAPF) is the subproblem of producing "
     "collision-free timed routes for all robots, with assignment and "
     "sequence fixed. Makespan is the time from the release of the first "
     "robot until the last robot is parked at the depot."),
    ("h3", "4.2 Problem Formulation"),
    ("p", "The testbed is modelled as a graph whose nodes are the lattice "
     "nodes, bay entries, workstations, node 0 and the depot slots and "
     "entries, and whose edges are the drivable segments between them. A "
     "job is tied to one workstation and a processing duration. A robot "
     "drops the job, which starts processing; the robot continues to other "
     "work, and a second visit picks the job up once processing has "
     "elapsed. When the same robot drops and picks up at one bay with no "
     "other work in between, the two visits are merged into one service "
     "visit, avoiding a banned mid-grid 180 degree turn."),
    ("lead", "Decision variables.", "Job-to-robot assignment; job "
     "sequence within each robot; route of each robot through the graph; "
     "and timing, meaning departure time and any planned waits."),
    ("lead", "Parameters.", "Grid topology; depot layout; fleet size; job "
     "set and workstations; processing duration (30 s); robot capacity (2 "
     "jobs); measured travel and turn times; and job release (all jobs "
     "known at the start)."),
    ("lead", "Constraints.", "Precedence (a pickup only after processing "
     "has elapsed); capacity; node, edge and swap conflicts within the "
     "collision windows (5 s at nodes, 7 s on edges); rotation clearance; "
     "deadlock freedom; and depot entry through node 0."),
    ("note", "Add the full mathematical model: sets, parameters, decision "
     "variables, objective (min makespan) and constraints, written as an "
     "MILP or as the bi-level structure of Ding et al. (2025). This is "
     "needed before the joint pipeline is finalized."),
    ("h3", "4.3 Planning Methods"),
    ("p", "All experiments use a sequential pipeline. The BRKGA solver "
     "allocates and orders jobs once, and the result is frozen. A traffic "
     "planner then makes the routes conflict-free. Four planners were "
     "compared:"),
    ("lead", "Uncoordinated A*.", "The baseline. Each robot gets its "
     "shortest path, ignoring the others, with no planned waits. Conflicts "
     "are left to the supervisor at run time."),
    ("lead", "WAIT.", "Fixed L-shaped paths. Find the first conflict, delay "
     "one robot, and repeat."),
    ("lead", "Prioritized planning (PP).", "Robots are planned one at a "
     "time with timed A* against a reservation table, longest predicted "
     "completion first. A robot can reroute as well as wait."),
    ("lead", "Conflict-based search (CBS).", "Plans robots alone, then "
     "branches on each conflict by constraining one robot or the other, "
     "keeping the cheapest conflict-free set (Sharon et al., 2015)."),
    ("p", "CBS is the only joint method among the four: its high-level "
     "search can change any robot's path to resolve a conflict. PP and WAIT "
     "are sequential, and A* is uncoordinated. All four share the testbed's "
     "timing model and conflict rules, and every planner was given a 30 s "
     "time limit."),
    ("fig", FIG + r"\fig_pipelines.png", "Planning pipelines: sequential "
     "(allocate, then route), sequential with feedback, and joint.", 6.0),
    ("h3", "4.4 Experiment 1: Traffic-Planning Comparison"),
    ("h4", "4.4.1 Objective and Design"),
    ("p", "Experiment 1 compares the four traffic-planning methods, and "
     "asks whether their performance changes between four and six robots, "
     "on one fixed workload. It is a 4 x 2 full factorial (method x fleet "
     "size) blocked by execution round: five rounds, each containing all "
     "eight treatments once in a randomized order (seed 20261019), for 40 "
     "runs."),
    ("h4", "4.4.2 Workload and Controlled Variables"),
    ("p", "The workload is 12 bays (B1, B5, B9, B10, B14, B17, B22, B24, "
     "B27, B30, B31, B42) with capacity 2, pickup and delivery enabled, 30 s "
     "processing and the cascade launch. The allocation was solved once per "
     "fleet size by BRKGA (30 s) and frozen. In the six-robot plan, two "
     "cross-robot pickups were removed so that every robot serves its own "
     "bays; the four-robot plan needed no change. Each method's routes were "
     "planned once and saved, so that repeats of a treatment drive "
     "identical routes. Drive mode (Camera + Encoder), node coordinates, "
     "per-robot calibration and supervisor rules were frozen for the "
     "campaign. The run deadline was twice the PP planned makespan."),
    ("h4", "4.4.3 Responses"),
    ("p", "The primary responses were completion rate (completed runs over "
     "all attempts) and actual makespan for completed runs. Secondary "
     "responses were prediction error (actual minus planned makespan), "
     "runtime traffic waits, collision-guard interventions, deadlocks, the "
     "spread of robot finish times and job flow time. Battery state, pose "
     "rate, camera outages and video were recorded as nuisance variables."),
    ("h4", "4.4.4 Infrastructure-Fault Rule"),
    ("p", "Declared before the first run: a failed run counts as an "
     "infrastructure fault if it failed because of a dropped robot session, "
     "a camera or pose outage (10 or more pose gaps over 300 ms, or any gap "
     "over 2 s), robot hardware, an operator setup error, or a robot not "
     "executing what the supervisor commanded (a turn accepted but never "
     "started, or a robot still moving after STOP). A deadlock or collision "
     "that follows such a fault is part of the fault. Infrastructure faults "
     "were re-run immediately and both rows kept; results are reported with "
     "and without them."),
    ("h3", "4.5 Statistical Analysis"),
    ("p", "Makespan was analysed with a randomized complete block design: "
     "y = round + robots + method + robots x method + error, with 28 error "
     "degrees of freedom, and with a blocked one-way model within each "
     "fleet size (12 error degrees of freedom). Pairwise differences used "
     "Tukey's HSD on the blocked error term. Because the residuals were not "
     "normal, the Friedman test (blocked by round) is reported next to every "
     "F test. Equivalence was tested with two one-sided tests at a 10 s "
     "margin, chosen after the data were seen and reported as such."),
    ("h3", "4.6 Experiment 2: Joint versus Sequential Planning"),
    ("note", "To be written. Planned design: one allocation and routing "
     "algorithm in two forms, sequential (allocate on free-space travel "
     "times, then route) and joint (each candidate scored by its conflict-"
     "aware makespan with PP in the loop). ALNS is the preferred search "
     "because it evaluates one candidate per iteration and already keeps "
     "explicit tours; sequential BRKGA is kept as the reference. Use more "
     "workstations and cross-robot pickups at a fixed fleet size, so that "
     "congestion is high enough for joint search to matter. Specify the "
     "instances, replications, equal wall-clock budgets and the physical "
     "subset."),
]

# --------------------------------------------------------------------------
CH5 = [
    ("h3", "5.1 Execution Reliability"),
    ("p", "Running six robots unattended exposed failure modes that are "
     "invisible with one robot. Each was diagnosed by measurement, and in "
     "four cases the first attribution was to the wrong subsystem."),
    ("lead", "Dropped commands.", "Best-effort commands could be lost "
     "silently. A BUSY acknowledgment and re-send gave 38 of 38 rotations "
     "acknowledged on first send."),
    ("lead", "Callback starvation.", "With six navigators sharing one "
     "process, spinning executors only inside motion primitives starved "
     "two robots of vision, and they collided. A dedicated spin thread per "
     "navigator restored 33.8-35.4 Hz for all six."),
    ("lead", "Transport-level command stalls.", "Robots aborted when the "
     "firmware watchdog saw no wheel setpoint for 1 s. Network bandwidth, "
     "CPU load, navigator stalls and the WiFi link were each ruled out by "
     "measurement; losses arrived simultaneously across robots, pointing at "
     "the shared micro-ROS agent. Moving the agent to the Linux laptop "
     "removed them."),
    ("lead", "Rotation geometry.", "A robot at a bay entry cannot rotate "
     "without overlapping its neighbours (Table 3.1). Contact distance is "
     "now computed from both robots' headings, and rotation is gated "
     "separately."),
    ("lead", "Deadlock detection.", "Every gate that makes a robot wait "
     "must register a wait-for edge, or cycle detection is blind. With "
     "detection and single-robot resolution in place, every run with zero "
     "declined resolutions completed."),
    ("lead", "Depot launch.", "Replacing fixed release offsets with the "
     "cascade launch cut the time to put the last robot on the grid from "
     "120.0 s to 69.3 s in a live mission, and cut planned makespan for the "
     "standard six-robot instance from 248.8 s to 184.2 s."),
    ("fig", FIG + r"\fig_launch_stages.png", "Sequential versus cascade "
     "depot launch.", 6.0),
    ("note", "Expand each finding into a subsection with its evidence "
     "(source: PROGRESS_REPORT_2026-10 sections 3-7, FINDINGS_ROTATION, "
     "FINDINGS_TURN). Add the Alvik1 no-motion turn analysis (clockwise "
     "turns, 20 of 181 first attempts)."),

    ("h3", "5.2 Experiment 1: Completion"),
    ("p", "All 40 treatment runs completed all 12 jobs. Of 44 real attempts "
     "(false starts excluded), 40 completed (91 percent). All four failures "
     "were infrastructure faults under the declared rule: two execution "
     "defects fixed during the first round, a command-link delay on Alvik1, "
     "and robot non-execution followed by a deadlock. Excluding them, every "
     "method completed 100 percent of its attempts. No collisions occurred "
     "in any run."),
    ("h3", "5.3 Experiment 1: Makespan"),
    ("p", "Fleet size dominated makespan: six robots finished in 253.9 s "
     "on average against 288.1 s for four (p < 0.001; Table 5.2). The method "
     "effect depended on fleet size. With six robots the method mattered (F "
     "= 10.76, p = 0.001; Friedman p = 0.014): WAIT was the slowest (260.2 "
     "s) and significantly slower than A* (253.0 s), PP (248.5 s) and CBS "
     "(253.9 s). PP was the fastest, but its lead over A* and CBS was not "
     "significant, and A* and CBS were statistically equivalent within 10 "
     "s. With four robots no method differed on average (p = 0.22), yet A* "
     "was the fastest method in all five rounds (Friedman p = 0.029). The "
     "method x robots interaction was suggestive but not significant (p = "
     "0.10)."),
    ("table", "Actual makespan by treatment (5 runs per cell).",
     ["Robots", "Method", "Mean (s)", "SD (s)", "Min (s)", "Max (s)",
      "Planned (s)"],
     [["4", "A*", "279.5", "3.6", "274.1", "283.0", "263.6"],
      ["4", "WAIT", "288.3", "9.9", "281.3", "305.7", "266.3"],
      ["4", "PP", "295.6", "26.7", "281.1", "343.1", "263.6"],
      ["4", "CBS", "289.1", "9.3", "281.6", "299.8", "263.6"],
      ["6", "A*", "253.0", "2.6", "249.3", "256.1", "230.7"],
      ["6", "WAIT", "260.2", "2.5", "256.3", "263.1", "235.5"],
      ["6", "PP", "248.5", "2.8", "244.9", "252.7", "230.7"],
      ["6", "CBS", "253.9", "4.1", "251.3", "261.1", "230.7"]]),
    ("table", "Analysis of variance for actual makespan (rounds as "
     "blocks).", ["Source", "SS", "df", "MS", "F", "p", "Partial eta sq."],
     [["Round", "1107.7", "4", "276.9", "2.89", "0.040", "0.29"],
      ["Robots", "11697.6", "1", "11697.6", "122.15", "<0.001", "0.81"],
      ["Method", "344.0", "3", "114.7", "1.20", "0.329", "0.11"],
      ["Robots x Method", "659.8", "3", "219.9", "2.30", "0.099", "0.20"],
      ["Error", "2681.4", "28", "95.8", "", "", ""],
      ["Total", "16490.5", "39", "", "", "", ""]]),
    ("fig", DOEFIG + r"\fig1_makespan_by_method.png", "Actual makespan "
     "by method: individual runs (rounds 1-5, left to right), mean with 95% "
     "confidence interval, and planned makespan (dashed).", 6.2),
    ("fig", DOEFIG + r"\fig2_interaction.png", "Method x fleet size: mean "
     "makespan with 95% confidence intervals.", 4.6),
    ("h3", "5.4 Experiment 1: Secondary Responses"),
    ("lead", "Prediction error.", "Every method ran 16-32 s (6-12 percent) "
     "longer than planned. Neither method (p = 0.54) nor fleet size (p = "
     "0.55) changed it: the timing model underestimates by a roughly "
     "constant amount, the same for every planner."),
    ("lead", "Waiting.", "Total robot-seconds of runtime waiting did not "
     "differ by method. With six robots PP held robots more often (17.2 "
     "stops against 10.6-12.6, p = 0.018) but for less time per stop (24.6 "
     "s against 36-41 s, p = 0.027)."),
    ("lead", "Who waits.", "PP narrowed the gap between the first and "
     "last robot home with six robots (62 s against 79-85 s, p = 0.002). It "
     "delayed the robots with slack (Alvik1, Alvik2) so that the robot that "
     "sets the makespan (Alvik4) finished sooner (Figure 5.4)."),
    ("lead", "Collision-guard interventions.", "Interventions were rare "
     "(51 in 40 runs, 0-3 per run) and did not depend on method (p = "
     "0.77). Uncoordinated A*, which leaves conflicts to the supervisor, "
     "needed no more interventions than the planners. There were more with "
     "four robots than six (1.6 against 1.0 per run, p = 0.012)."),
    ("fig", DOEFIG + r"\fig3_robot_finish_6r.png", "Six robots: finish "
     "time of each robot by method (mean and 95% confidence interval).",
     6.0),
    ("note", "Investigate and explain why guard interventions were more "
     "frequent with four robots (which robot pairs trigger them)."),
    ("h3", "5.5 Discussion of Experiment 1"),
    ("p", "With this workload (12 bays, allocation frozen, each robot "
     "serving its own bays), the choice of traffic planner matters little: "
     "the largest difference between methods was 11.7 s (4.7 percent). WAIT "
     "is consistently slower with six robots because its fixed L-shaped "
     "returns send every robot along the bottom row and add turns. PP's "
     "advantage comes from execution, not from the plan: its planned "
     "makespan equals that of A* and CBS. The joint planner (CBS) did not "
     "beat the sequential planner (PP) on any response; with few conflicts "
     "there is little for joint search to win."),
    ("p", "Execution noise was as large as the method effects. With four "
     "robots PP and CBS ran the identical frozen plan, yet their runs took "
     "281-343 s. The longest run, DOE_4r_pp_2 (343.1 s), completed within "
     "the deadline and was kept as declared; Alvik1's turns failed to "
     "start, holding it about 60 s at a workstation. Replacing that run "
     "with its cell median would make the overall method effect significant "
     "(p = 0.33 to 0.002), which is why rank-based tests are reported "
     "alongside the F tests."),
    ("note", "Add threats to validity: a single workload and topology, "
     "a fixed allocation, the post hoc equivalence margin, and five "
     "replications per cell."),
    ("h3", "5.6 Experiment 2: Joint versus Sequential Planning"),
    ("note", "Results of the follow-up experiment go here."),
]

# --------------------------------------------------------------------------
CH6 = [
    ("h3", "6.1 Summary"),
    ("p", "This thesis built a six-robot AGV testbed with overhead AprilTag "
     "localization and a centralized ROS 2 supervisor, and used it to "
     "compare four traffic-planning methods in a replicated, blocked "
     "experiment."),
    ("h3", "6.2 Conclusions"),
    ("p", "Fleet size dominated makespan. The traffic planner mattered only "
     "at six robots, where WAIT was slowest and PP fastest; PP's advantage "
     "arose in execution rather than in its plan. Every method ran longer "
     "than planned by a similar amount, and execution faults contributed "
     "variation as large as the differences between methods."),
    ("note", "Add the conclusion to the primary research question once "
     "Experiment 2 is complete."),
    ("h3", "6.3 Limitations"),
    ("note", "Single testbed and topology; six robots; one workload in "
     "Experiment 1; allocation frozen; execution faults concentrated on one "
     "robot; camera-dependent localization."),
    ("h3", "6.4 Future Work"),
    ("note", "Joint pipeline at larger scale; ALNS versus BRKGA; event-"
     "based (action-dependency) execution in place of timed waits; return-"
     "to-depot rework; a timing model corrected for the measured 6-12 "
     "percent underestimate; vision-language dispatch advisor (Appendix B)."),
]

APPENDICES = [
    ("h3", "Appendix A: Measured Constants"),
    ("table", "Depot lane and launch timing (2026-09-15, six robots).",
     ["Constant", "Value"],
     [["Slot to entry node", "2.22 s"], ["Turn at entry node", "1.94 s"],
      ["Lane hop, entry to entry", "1.74 s drive + 0.38 s settle"],
      ["Entry to node 0", "1.82 s"], ["Turn at node 0", "1.90 s"],
      ["Node 0 to node 1", "3.78 s"], ["Node-0 headway floor", "5.68 s"]]),
    ("table", "Robot and table geometry (2026-09-30).", ["Constant",
     "Value"],
     [["Robot, corner to corner", "5.625 in"], ["Robot top face",
      "4.0 x 4.0 in"], ["Lattice pitch", "10.00 in"],
      ["Bay entry to adjacent lattice node", "5.00 in"],
      ["Bay entry to its workstation", "4.99 in"],
      ["Depot slot pitch", "6.30-6.50 in"], ["Table", "97 x 97 in"],
      ["Reference tag black square", "3.835 in"],
      ["Node 1 position", "(13.5, 16.75) in"]]),
    ("h3", "Appendix B: Vision-Language Dispatch Advisor Study"),
    ("note", "Thirteen local models screened, a 24-block randomized "
     "complete block design over 2,012 questions, the overlay and JPEG-"
     "quality factors, and the latency-tail decision. Source: "
     "FINDINGS_VLM_2026-09-22 and PROGRESS_REPORT_2026-10 section 8. Cut "
     "first if length requires."),
    ("h3", "Appendix C: Experiment 1 Run Order and Data"),
    ("note", "Insert the run order (DOE_RUN_ORDER.txt) and the per-run "
     "results (DOE_traffic_planning_results.xlsx, tab 'Runs')."),
    ("h3", "Appendix D: Software and Hardware Repository"),
    ("note", "Repository link, directory layout, and how to reproduce a "
     "run and the analysis (build_doe_dataset.py, analyze_doe.py, "
     "plot_doe.py, build_doe_workbook.py)."),
]

BIB = [
    "Berbeglia, G., Cordeau, J.-F., Gribkovskaia, I., and Laporte, G., "
    "\u201cStatic pickup and delivery problems: a classification scheme and "
    "survey,\u201d TOP, 15(1), 2007, pp. 1-31.",
    "Chen, Z., Alonso-Mora, J., Bai, X., Harabor, D. D., and Stuckey, P. J., "
    "\u201cIntegrated task assignment and path planning for capacitated "
    "multi-agent pickup and delivery,\u201d IEEE Robotics and Automation "
    "Letters, 6(3), 2021, pp. 5816-5823.",
    "Ding, P., Liu, S. Q., Chung, S.-H., Masoud, M., and Zhang, Q., \u201cA "
    "bi-level programming approach for coordinated task sequencing and "
    "collision-free path planning in robotic mobile fulfillment systems,"
    "\u201d Mathematics, 13(23), 2025, 3783.",
    "Erdmann, M., and Lozano-Perez, T., \u201cOn multiple moving objects,"
    "\u201d Algorithmica, 2, 1987, pp. 477-521.",
    "Goncalves, J. F., and Resende, M. G. C., \u201cBiased random-key genetic "
    "algorithms for combinatorial optimization,\u201d Journal of Heuristics, "
    "17(5), 2011, pp. 487-525.",
    "Huang, T., Li, J., Koenig, S., and Dilkina, B., \u201cAnytime multi-agent "
    "path finding via machine learning-guided large neighborhood search,"
    "\u201d Proceedings of the AAAI Conference on Artificial Intelligence, "
    "36(9), 2022, pp. 9368-9376.",
    "Li, J., Chen, Z., Harabor, D., Stuckey, P. J., and Koenig, S., "
    "\u201cMAPF-LNS2: Fast repairing for multi-agent path finding via large "
    "neighborhood search,\u201d Proceedings of the AAAI Conference on "
    "Artificial Intelligence, 36(9), 2022, pp. 10256-10265.",
    "Li, J., Tinka, A., Kiesel, S., Durham, J. W., Kumar, T. K. S., and "
    "Koenig, S., \u201cLifelong multi-agent path finding in large-scale "
    "warehouses,\u201d Proceedings of the AAAI Conference on Artificial "
    "Intelligence, 35(13), 2021, pp. 11272-11281.",
    "Ma, H., Li, J., Kumar, T. K. S., and Koenig, S., \u201cLifelong multi-"
    "agent path finding for online pickup and delivery tasks,\u201d "
    "arXiv:1705.10868, 2017.",
    "Macenski, S., Foote, T., Gerkey, B., Lalancette, C., and Woodall, W., "
    "\u201cRobot Operating System 2: Design, architecture, and uses in the "
    "wild,\u201d Science Robotics, 7(66), 2022, eabm6074.",
    "Olson, E., \u201cAprilTag: A robust and flexible visual fiducial "
    "system,\u201d in Proceedings IEEE International Conference on Robotics "
    "and Automation, 2011, pp. 3400-3407.",
    "Parragh, S. N., Doerner, K. F., and Hartl, R. F., \u201cA survey on "
    "pickup and delivery problems,\u201d Journal fur Betriebswirtschaft, "
    "58(1), 2008, pp. 21-51.",
    "Phan, T., Huang, T., Dilkina, B., and Koenig, S., \u201cAdaptive anytime "
    "multi-agent path finding using bandit-based large neighborhood "
    "search,\u201d arXiv:2312.16767, 2024.",
    "Ropke, S., and Pisinger, D., \u201cAn adaptive large neighborhood search "
    "heuristic for the pickup and delivery problem with time windows,\u201d "
    "Transportation Science, 40(4), 2006, pp. 455-472.",
    "Saseendran, H., Sodhi, M., and Prasad, R., \u201cVehicle routing problem "
    "with resource-constrained pickup and delivery,\u201d arXiv:2602.23685, "
    "University of Rhode Island, 2026.",
    "Sharon, G., Stern, R., Felner, A., and Sturtevant, N. R., \u201cConflict-"
    "based search for optimal multi-agent pathfinding,\u201d Artificial "
    "Intelligence, 219, 2015, pp. 40-66.",
    "Silver, D., \u201cCooperative pathfinding,\u201d in Proceedings of the "
    "AAAI Conference on Artificial Intelligence and Interactive Digital "
    "Entertainment, 2005, pp. 117-122.",
    "Stern, R., Sturtevant, N., Felner, A., Koenig, S., Ma, H., Walker, T., "
    "Li, J., Atzmon, D., Cohen, L., Kumar, T. K. S., Bartak, R., and "
    "Boyarski, E., \u201cMulti-agent pathfinding: Definitions, variants, and "
    "benchmarks,\u201d Proceedings of the International Symposium on "
    "Combinatorial Search, 10(1), 2021, pp. 151-158.",
    "Wang, J., and Olson, E., \u201cAprilTag 2: Efficient and robust fiducial "
    "detection,\u201d in Proceedings IEEE/RSJ International Conference on "
    "Intelligent Robots and Systems, 2016, pp. 4193-4198.",
    "Wurman, P. R., D\u2019Andrea, R., and Mountz, M., \u201cCoordinating "
    "hundreds of cooperative, autonomous vehicles in warehouses,\u201d AI "
    "Magazine, 29(1), 2008, pp. 9-20.",
    "Yang, X., Hua, G., Zhang, L., Cheng, T. C. E., and Choi, T. M., "
    "\u201cJoint order assignment and picking station scheduling in KIVA "
    "warehouses with multiple stations,\u201d arXiv:2108.09056, 2023.",
    "Zhang, H., Wu, Y., Hu, J., and Wang, Y., \u201cCollaborative "
    "optimization of task scheduling and multi-agent path planning in "
    "automated warehouses,\u201d Complex & Intelligent Systems, 9(5), 2023, "
    "pp. 5937-5948.",
]

CHAPTERS = [("INTRODUCTION", CH1), ("REVIEW OF LITERATURE", CH2),
            ("TESTBED AND SYSTEM DESIGN", CH3), ("METHODOLOGY", CH4),
            ("FINDINGS", CH5), ("CONCLUSION", CH6)]
