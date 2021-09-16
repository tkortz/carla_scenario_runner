Project Information
===============

This fork of the [CARLA Scenario Runner](https://github.com/carla-simulator/scenario_runner) was used for an extended version of the paper ["The Price of Schedulability in Multi-ObjectTracking: The History-vs.-Accuracy Trade-Off"](https://www.cs.unc.edu/~anderson/papers/isorc20.pdf), originally presented at presented at ISORC 2020.  The fork for the original paper can be found [in the `isorc20` branch](https://github.com/Yougmark/scenario_runner/tree/isorc20).

The primary instructions for running our experiments are available in [our experiment-running repository](https://github.com/tkortz/isorc20_experiments/tree/journal).

---

The original CARLA Scenario Runner README follows.

ScenarioRunner for CARLA
========================
This repository contains traffic scenario definition and an execution engine
for CARLA. It also allows  the execution of a simulation of the carla challenge.
You can use this system to prepare your agent for the CARLA challenge.

Disclaimer
----------

The current status is work in progress and may not reflect the final API

Building the ScenarioRunner
---------------------------

Use `git clone` or download the project from this page. Note that the master
branch contains the latest fixes and features, and my require to use the latest features from CARLA.

Currently no build is required, as all code is in Python.


Using the ScenarioRunner
------------------------

Please take a look at our [Getting started](Docs/getting_started.md)
documentation.

Challenge Evaluation
---------------------

You can evaluate your own agents using a reproduction
 of the CARLA challenge by following [this tutorial](Docs/challenge_evaluation.md)



Contributing
------------

Please take a look at our [Contribution guidelines][contriblink].

[contriblink]: http://carla.readthedocs.io/en/latest/CONTRIBUTING

F.A.Q.
------

If you run into problems, check our
[FAQ](http://carla.readthedocs.io/en/latest/faq/).

License
-------

ScenarioRunner specific code is distributed under MIT License.
