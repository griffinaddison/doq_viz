import xacro
import argparse

parser = argparse.ArgumentParser(description="Xacro wrapper for producing mujoco compatible xml")
parser.add_argument("filename", type=str, help="File to be converted")
parser.add_argument("-o", "--output", type=str, help="File to be converted")
args = parser.parse_args()


xml_string = xacro.process_file(args.filename)\
                .toprettyxml(indent='  ').replace(" xmlns:xacro=\"http://www.ros.org/wiki/xacro\"", "")
if args.output:
  with open(args.output, "w") as file:
    file.write(xml_string)
else:
  print(xml_string)

