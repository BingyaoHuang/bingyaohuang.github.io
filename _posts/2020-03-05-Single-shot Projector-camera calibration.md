---
layout: project
title: A Single-shot-per-pose camera-projector calibration system for imperfect planar targets
author: Bingyao Huang
permalink: /single-shot-pro-cam-calib
published: true
teaser: https://raw.githubusercontent.com/BingyaoHuang/single-shot-pro-cam-calib/master/doc/reconstruct.png
---

#### Bingyao Huang, Samed Ozdemir, Ying Tang, Chunyuan Liao and [Haibin Ling](https://www3.cs.stonybrook.edu/~hling)
#### ISMAR-Adjunct 2018
#### [[Paper](https://arxiv.org/pdf/1803.09058.pdf)]  [[GitHub](https://github.com/BingyaoHuang/single-shot-pro-cam-calib)]

<p align="center"><img src="https://raw.githubusercontent.com/BingyaoHuang/single-shot-pro-cam-calib/master/doc/reconstruct.png" alt="procam-calib" width="80%"/></p>

## Abstract
Existing camera-projector calibration methods typically warp feature points from a camera image to a projector image using estimated
homographies, and often suffer from errors in camera parameters
and noise due to imperfect planarity of the calibration target. In
this paper we propose a simple yet robust solution that explicitly
deals with these challenges. Following the structured light (SL)
camera-project calibration framework, a carefully designed correspondence algorithm is built on top of the De Bruijn patterns. Such
correspondence is then used for initial camera-projector calibration.
Then, to gain more robustness against noises, especially those from
an imperfect planar calibration board, a bundle adjustment algorithm
is developed to jointly optimize the estimated camera and projector models. Aside from the robustness, our solution requires only
one shot of SL pattern for each calibration board pose, which is
much more convenient than multi-shot solutions in practice. Data
validations are conducted on both synthetic and real datasets, and
our method shows clear advantages over existing methods in all
experiments

___
## Single-shot-per-pose
<p align="center"><img src="https://raw.githubusercontent.com/BingyaoHuang/single-shot-pro-cam-calib/master/doc/calib.gif" alt="calib" width="80%"/></p>

___
## Software video user guide
<!-- <p align="center"> -->
<div class="video_wrapper">
<div class="responsive-video">
<iframe width="80%" height="400px" src="https://www.youtube.com/embed/fnrVDOhcu7I?mute=1" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>
</div>

___