# my-ros2-codecov-exp

[![codecov](https://codecov.io/gh/TommyChangUMD/my-ros2-codecov-exp/branch/main/graph/badge.svg?token=KRAHD3BZP7)](https://codecov.io/gh/TommyChangUMD/my-ros2-codecov-exp)

![CICD Workflow status](https://github.com/TommyChangUMD/my-ros2-codecov-exp/actions/workflows/my_codecov_upload.yml/badge.svg)

This repo provides a template for:

  - GitHub CI
    - "main" branch installs ROS 2 Humble on top of Ubuntu 22.04
    - "test-docker" branch uses a ROS 2 Galactic docker container from docker hub
  - Codecov
  - Build ROS 2 package
  - Build C++ library 
  - Subscribe to multiple topics (no need to use multiple callbacks.)
  - ROS2 node unit test (see [minimal_integration_test](https://github.com/TommyChangUMD/minimal_integration_test), [level 2 unit test ](https://www.theconstructsim.com/how-to-test-your-ros-programs/))

## How to build for test coverage

```
rm -rf build/cpp_pubsub/
colcon build --cmake-args -DCOVERAGE=1 --packages-select cpp_pubsub
cat log/latest_build/cpp_pubsub/stdout_stderr.log
```

## How to run unit tests

```
source install/setup.bash
colcon test --packages-select cpp_pubsub
cat log/latest_test/cpp_pubsub/stdout_stderr.log
```

You should see:

```
: [==========] Running 1 test from 1 test suite.
1: [----------] Global test environment set-up.
1: [----------] 1 test from TaskPlanningFixture
1: [ RUN      ] TaskPlanningFixture.TrueIsTrueTest
1: [INFO] [1670717396.964270896] [basic_test]: DONE WITH CONSTRUCTOR!!
1: [INFO] [1670717398.859776334] [basic_test]: DONE WITH SETUP!!
1: TEST BEGINNING!!
1: [INFO] [1670717399.361518186] [basic_test]: I heard: 'Hello, world! 3'
1: [INFO] [1670717399.861460092] [basic_test]: I heard: 'Hello, world! 4'
1: [INFO] [1670717400.361478619] [basic_test]: I heard: 'Hello, world! 5'
1: [INFO] [1670717400.861412773] [basic_test]: I heard: 'Hello, world! 6'
1: [INFO] [1670717401.361391912] [basic_test]: I heard: 'Hello, world! 7'
1: DONE WITH TEARDOWN
1: [       OK ] TaskPlanningFixture.TrueIsTrueTest (4961 ms)
1: [----------] 1 test from TaskPlanningFixture (4961 ms total)
1: 
1: [----------] Global test environment tear-down
1: [==========] 1 test from 1 test suite ran. (4961 ms total)
1: [  PASSED  ] 1 test.
1: DONE SHUTTING DOWN ROS

....

Label Time Summary:
cppcheck      =   0.18 sec*proc (1 test)
gtest         =   5.17 sec*proc (1 test)
lint_cmake    =   0.19 sec*proc (1 test)
linter        =   1.24 sec*proc (5 tests)
pep257        =   0.25 sec*proc (1 test)
uncrustify    =   0.23 sec*proc (1 test)
xmllint       =   0.39 sec*proc (1 test)

Total Test time (real) =   6.41 sec
```

## How generate code coverage report (both lcov info file and html output)
```
source install/setup.bash
ros2 run cpp_pubsub generate_coverage_report.bash
```

You should see:
```
Summary coverage rate:
  lines......: 100.0% (29 of 29 lines)
  functions..: 100.0% (6 of 6 functions)
  branches...: no data found
Reading data file /home/tchang/proj/my-ros2-codecov-exp/install/cpp_pubsub/lib/cpp_pubsub/coverage_cleaned.info
Found 2 entries.
Found common filename prefix "/home/tchang/proj/my-ros2-codecov-exp/cpp_pubsub"
Writing .css and .png files.
Generating output.
Processing file src/publisher_member_function.cpp
Processing file src/subscriber_member_function.cpp
Writing directory view page.
Overall coverage rate:
  lines......: 100.0% (29 of 29 lines)
  functions..: 100.0% (6 of 6 functions)
Code Coverage generated:
     /home/tchang/proj/my-ros2-codecov-exp/install/cpp_pubsub/lib/cpp_pubsub/coverage_cleaned.info
     /home/tchang/proj/my-ros2-codecov-exp/install/cpp_pubsub/coverage/index.html
```

You can take a look at the generated report now.  For example:

```
firefox /home/tchang/proj/my-ros2-codecov-exp/install/cpp_pubsub/coverage/index.html
```
[<img src=screenshots/Screenshot-2022-12-07-023731.png 
    width="70%" 
    style="display: block; margin: 0 auto"
    />](screenshots/Screenshot-2022-12-07-023731.png)


## How to use GitHub CI to upload coverage report to Codecov

### First, sign up Codecov with you GitHub account.  

  https://about.codecov.io/sign-up/

### Enable the repository you want to upload from

After you sign in, you should see a list of your repositories (you may
have to refresh and reload the page a few times). Enable the one you
want to receive coverage data from.

### Create a GitHub CI yaml file

See below for the setup of this repo:

https://github.com/TommyChangUMD/my-ros2-codecov-exp/blob/main/.github/workflows/my_codecov_upload.yml

### Add your Codecov and GitHub CI badge to README.md

Follow the instruction below to copy the badge (in markdown format)
and paste it in your README.md

https://docs.codecov.com/docs/status-badges

https://docs.github.com/en/actions/monitoring-and-troubleshooting-workflows/adding-a-workflow-status-badge

Note: When you click on the codecov badge, you should see the coverage
report.  You should also see the source file listing.  If not, you may
need to login your codecov account first.

[<img src=screenshots/Screenshot-2022-12-07-164405.png
    width="70%" 
    style="display: block; margin: 0 auto"
    />](screenshots/Screenshot-2022-12-07-164405.png)

[<img src=screenshots/Screenshot-2022-12-07-164423.png
    width="70%"
    style="display: block; margin: 0 auto"
    />](screenshots/Screenshot-2022-12-07-164423.png)

