# Run Offline Demo

Apollo provides a method to run simulation if you do not have the required
hardware.

First fork and then clone Apollo's GitHub code and then set up the docker release environment by following the instructions in the
[Install docker](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_build_and_release.md#docker)
section of the
[Build and Release](https://github.com/ApolloAuto/apollo/blob/master/docs/howto/how_to_build_and_release.md)
page.

Setup steps:

1. Start the docker release environment using the command:

    ```
    bash docker/scripts/dev_start.sh
    ```

2. Enter the docker release environment:

    ```
    bash docker/scripts/dev_into.sh
    ```

3. Build Apollo in the Container:
    ```
    bash apollo.sh build
    ```
    `Note:` If you do not have a GPU, you can use the following script instead

    ```
    bash apollo.sh build_cpu
    ```
4. Bootstrap to start ros call and Monitor module and Dreamview
    ```
    bash scripts/bootstrap.sh
    ```

5. Now you can play the rosbag:

    ```
    sudo python docs/demo_guide/rosbag_helper.py demo_2.0.bag #download rosbag
    rosbag play demo_2.0.bag --loop
    ```

    The `--loop` option enables rosbag to keep playing the bag in a loop
    playback mode.

6. Open Chrome and go to **localhost:8888** to access Apollo Dreamview, which
   opens the screen below.
    ![](images/dv_trajectory.png)
   The car in Dreamview is happy to move around!

Congratulations!
