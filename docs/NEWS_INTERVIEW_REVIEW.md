# BlindNav News Interview Review Sheet

## One-Sentence Summary

BlindNav is a wearable AI navigation assistant for blind users that combines
real-time object detection, depth sensing, motion awareness, and voice alerts
to help a user understand nearby obstacles while walking.

## 20-Second Version

I built a chest-mounted navigation system that runs on a Raspberry Pi 4 with an
Intel RealSense depth camera. It detects people and obstacles, estimates how
threatening they are based on distance and motion, and speaks short left/right/
ahead warnings through headphones. It also supports on-demand AI scene
description.

## 60-Second Version

The project started from a simple assistive-tech question: can I build
something that helps a blind user understand the space directly in front of
them while moving? The current system uses a Raspberry Pi 4, an Intel RealSense
D435 depth camera, an IMU, YOLO object detection, depth-based distance
measurement, ego-motion compensation, and local Piper text-to-speech. The goal
is not to replace a cane or a guide dog. The goal is to provide extra spatial
awareness by warning about nearby people and obstacles in a way that is timely,
brief, and usable while walking.

## What Problem It Solves

- Standard obstacle detection alone is often too blunt: it can tell you that
  something exists, but not whether it matters right now.
- Blind users need timing, direction, and confidence, not just labels.
- Indoor spaces are dynamic. People pass by, chairs move, tables stick out,
  and walls or glass may not be obvious to a standard RGB-only system.
- A good assistive system must avoid constant chatter. Silence is as important
  as speech.

## What Makes This Different

- It combines object detection and depth sensing, so it knows both what an
  object is and roughly how far away it is.
- It estimates threat, not just presence. A close object matters more than a
  far object. A moving person matters differently than a static chair.
- It tracks whether the user is moving, because navigation guidance should
  change if the user is standing still versus walking.
- It is designed around short voice output, not a screen.
- It includes hardware-free regression tests, so behavior can be improved
  between field runs without needing the camera every time.

## What Changed Since The ESP32 Stage

- The earlier ESP32 phase was much more limited and was closer to a sensor
  experiment than a full navigation assistant.
- The current system moved to a Raspberry Pi 4 so it can run camera-based
  computer vision, ONNX inference, depth processing, and local neural speech.
- The project shifted from "can I detect obstacles at all?" to "can I deliver
  useful, low-chatter, real-time guidance while walking?"
- The current version includes:
  - YOLO26n object detection
  - Intel RealSense D435 depth sensing
  - IMU-based movement awareness
  - ego-motion compensation
  - left/right/ahead voice guidance
  - on-demand AI scene description
  - a 172-test hardware-free regression suite

## Current Hardware Stack

- Raspberry Pi 4
- Intel RealSense D435
- ICM-20948 IMU
- Bluetooth headphones
- Chest harness

## Current Software Stack

- Python
- ONNX Runtime
- YOLO26n object detector
- Intel RealSense SDK
- Piper TTS
- Claude for on-demand scene description

## Core Runtime Pipeline

1. Capture synchronized RGB and depth frames from the RealSense camera.
2. Run YOLO object detection on the RGB frame.
3. Track objects across frames.
4. Sample depth inside each tracked box to estimate distance.
5. Estimate ego-motion so stationary objects do not look like they are moving
   just because the user is walking.
6. Score threats based on distance, motion, and context.
7. Choose whether to speak, what to say, and how urgently to say it.
8. Play a short voice alert through headphones.

## The Most Important Engineering Constraint

The hardest problem is not object detection by itself. The hardest problem is
deciding when to speak and when not to speak. If the system talks too much, it
becomes distracting. If it talks too little, it misses something important.

## Safety Philosophy

- This is an assistive prototype, not a replacement for a cane, guide dog, or
  formal Orientation and Mobility training.
- It should add awareness, not take control away from the user.
- It should be conservative about uncertainty.
- It should avoid pretending to know more than it really knows.
- It should fail quietly rather than sound overconfident when motion or depth
  data becomes unreliable.

## What v3.28 Added

- Bucketed spoken distances:
  warnings now snap to stable 30 cm voice buckets so repeated alerts reuse the
  same phrases instead of synthesizing many slightly different decimals.
- Better voice diagnostics:
  logs now separate queue wait, synthesis time, playback launch delay, and
  cache hit versus miss.
- Better side-pass person handling:
  people moving by on the left or right are promoted earlier, even when radial
  time-to-collision is weak.
- Better bad-ego handling:
  the system now suppresses far false motion-based alerts when ego-motion
  confidence is poor.

## Numbers To Remember

- Current version: `v3.28 HEADLESS`
- Main production script: `raspberry_pi/yolo_realsense_navigation.py`
- Current hardware-free validation: `172 passed`
- Camera: Intel RealSense D435
- Compute: Raspberry Pi 4

## What Is Validated Well

- Object tracking logic
- Distance sampling logic
- Voice cooldown logic
- Priority queue behavior
- Voice TTL and skip-ahead behavior
- Threat scoring truth tables
- Left/right/ahead classification logic
- Motion filtering and bad-ego suppression logic

## What Still Needs Real-World Testing

- Crowded scenes
- Long walking sessions
- Bluetooth timing under field conditions
- Thermal performance on the Pi
- User preferences for how much voice output is helpful

## Honest Limitations

- The system can still slow down when the Pi gets hot.
- A spoken warning can still be late if a phrase is already playing, because
  active audio playback is intentionally never hard-killed.
- Camera occlusion and depth noise still matter in real environments.
- It is still a prototype and should be described honestly as one.

## Good Ways To Describe The Project

- "A wearable AI navigation assistant prototype for blind users."
- "A system that combines depth sensing, computer vision, and voice alerts."
- "An attempt to make obstacle awareness more timely and less noisy."

## Bad Ways To Describe The Project

- "It solves blindness."
- "It replaces a white cane."
- "It is production-ready for unsupervised public deployment."
- "It always understands the environment correctly."

## Likely Interview Questions

### What inspired this project?

I became interested in accessibility technology through my grandmother's vision
loss and started asking what kind of system could actually be helpful in real
physical spaces, not just in a lab demo.

### Why not just use a phone app?

A phone is useful for many things, but navigation assistance while walking
works better when the sensing is hands-free, chest-mounted, and always aimed at
the path ahead.

### Why is depth sensing important?

Because object labels alone are not enough. A chair two meters away matters
very differently than a chair thirty centimeters away.

### Why does voice design matter so much?

Because a blind user cannot afford to be flooded with useless audio. The system
has to be brief, timely, and quiet when nothing important is happening.

### What was the hardest technical problem?

Balancing false positives and false negatives in a way that still feels usable
in motion. That includes motion compensation, deciding when a person passing by
matters, and reducing voice delay.

### What are you most proud of?

That it is not just a one-off demo. I built a real regression suite around it,
so I can improve behavior systematically instead of guessing between field
tests.

### Is it finished?

No. It is a serious prototype with strong software validation, but it still
needs continued field testing, refinement, and safety-focused iteration.

## Strong Soundbites

- "The challenge is not just seeing objects. The challenge is deciding when a
  spoken warning is actually useful."
- "For assistive technology, silence can be as important as speech."
- "A navigation system has to understand both the world and the user's motion
  through it."
- "I wanted to move from a sensor demo to something that behaves more like a
  real assistant."

## If Asked About AI

- The system uses AI in two different ways:
  - local vision for real-time detection
  - on-demand multimodal AI for richer scene description
- Real-time safety-relevant behavior should be short, deterministic, and
  low-latency.
- Longer open-ended descriptions are useful, but they are not the same as
  split-second walking guidance.

## If Asked About Impact

The goal is not to claim that one student project fixes a huge accessibility
problem. The goal is to contribute something real: a working prototype, a set
of engineering lessons, and open project documentation that can help future
assistive-tech work.

## Best Closing Line

This project is really about respecting the difference between detecting the
world and making that information usable in the moment for a blind traveler.
