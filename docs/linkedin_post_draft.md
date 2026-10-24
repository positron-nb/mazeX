# LinkedIn post (ready to paste)

Attach `media/showcase_18m_s3_weighted.mp4` (85 s, 1280×860, H.264; LinkedIn accepts it as-is).
Replace `<GitHub link>` after you push the repo. Every number below is from `results/`.

---

Can a robot escape a maze it has never seen, using only its own sensors? 🤖

I'm a 2nd-year ECE student without a physical robot, so I built the whole thing in simulation: MazeMind,
a ROS 2 + Gazebo project where a robot starts in a random maze with no map and no idea where the exit
is. It builds its map with LiDAR SLAM, decides where to explore, spots the exit marker with its camera,
and drives out.

My exploration algorithm: rank frontiers by visible information gain, travel cost, wall clearance,
path safety and a memory of failed or revisited goals, then navigate with Nav2. The twist I didn't
expect to need: the 360° LiDAR "finishes" mapping long before the forward camera has looked down
every corridor. My first version mapped 96 % of a maze and never saw the exit. So the robot now also
tracks what its *camera* has seen and explores that.

In the video it escapes an 18 m × 18 m maze (12 × 12 grid), travelling 170 m in 651 simulated seconds
with zero wall contacts and zero failed navigation goals.

I also benchmarked it properly: 60 runs, 3 strategies on identical mazes, every run verified against
ground truth.
• 6×6 mazes: informed frontier selection needed about half the distance of random selection (better on every maze)
• 4×4 mazes: my weighted score beat nearest-frontier on time in 10/10 mazes
• 6×6 mazes: weighted vs nearest-frontier was statistically a tie, and that's in the README too
• 0 wall contacts in all 60 runs

The best part was debugging with ground truth: false SLAM loop closures in look-alike corridors, a 9 %
odometry error from wheel collision geometry, and a camera that dropped 70 % of its frames because of
a TF timestamp issue. Each one was found by measuring before fixing.

Code, docs and every run's raw data: <GitHub link>

What would you add to the frontier score: a bias toward the outer walls, or something smarter?

#ROS2 #Robotics #SLAM #Nav2 #Gazebo #AutonomousNavigation #Python #OpenSource

---

## Before posting

- [ ] Watch the video once to the end. It ends on `OUTCOME: EXIT_REACHED`.
- [ ] Push the repo first (README GIF and results must be visible from the link).
- [ ] If you change any number, take it from `results/*/summary.md` or `results/showcase/runs/*.json`.
- [ ] Mention that the video is an 8× time-lapse if anyone asks; the caption in the README says so.
