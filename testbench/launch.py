import roslaunch
import rospy
import os
import shutil

def main():
  bags_root = '/bags'
  ros_root = '/root/.ros'
  results_root = '/data/orbslam-results'
  trajectory_file = '/root/.ros/KeyFrameTrajectory.txt'
  bags = [
    # 'vabadusepst',
    'vabadussild',
    'oldtown'
    ]

  rospy.init_node('en_Mapping', anonymous=True)
  uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
  roslaunch.configure_logging(uuid)

  # cli_args = ["bag.launch", 'bag_path:=/bags/test.bag']
  for bag in bags:
    bag_path = os.path.join(bags_root, bag + '.bag')
    out_dir = os.path.join(results_root, bag)
    print("*** Input file: {}".format(bag_path))
    for i in xrange(1, 6):
      print("*** Iteration: {}".format(i))
      cli_args = ["sekonix-mono.launch", 'bag_path:=' + bag_path]
      roslaunch_args = cli_args[1:]
      roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

      launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file, sigint_timeout=60, sigterm_timeout=60)

      launch.start()
      launch.spin()
      launch.shutdown()

      if not os.path.exists(out_dir):
        os.makedirs(out_dir)

      shutil.move(trajectory_file, os.path.join(out_dir, "it_{}.tum".format(i)))


if __name__ == "__main__":
    main()
