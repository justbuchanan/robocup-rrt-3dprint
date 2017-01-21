#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

#include <rrt/2dplane/GridStateSpace.hpp>
#include <rrt/BiRRT.hpp>

using namespace std;
using namespace RRT;
using namespace Eigen;

const float meter = 1000;

const float sizeScaleUp = 2;
const float RobotRadius = 0.09 * meter * sizeScaleUp;
const float RobotHeight = 0.15 * meter * sizeScaleUp;

const float scale = 0.6;
const float FieldLength = 6.05 * meter * scale;
const float FieldWidth = 4.05 * meter * scale;

const float LineHeight = 0.02 * meter;
const float LineWidth = 0.02 * meter;

class RobotStateSpace : public PlaneStateSpace<Vector2f> {
public:
  std::vector<Vector2f> robots;

  RobotStateSpace() : PlaneStateSpace(FieldWidth, FieldLength){};

  bool stateValid(const Vector2f &pt) const override {
    if (!PlaneStateSpace::stateValid(pt))
      return false;

    for (Vector2f r : robots) {
      float dist = (r - pt).norm();
      if (dist < 2 * RobotRadius)
        return false;
    }
    return true;
  }

  // TODO: these should be implemented in PlaneStateSpace
  // TODO: use override keyword in rrt lib

  bool transitionValid(const Vector2f &from,
                       const Vector2f &to) const override {
    return stateValid(from) && stateValid(to);
  }

  Vector2f intermediateState(const Vector2f &source, const Vector2f &target,
                             float stepSize) const override {
    Vector2f delta = target - source;
    delta = delta / delta.norm(); //  unit vector
    Vector2f val = source + delta * stepSize;
    return val;
  }

  virtual Vector2f intermediateState(const Vector2f &source,
                                     const Vector2f &target, float minStepSize,
                                     float maxStepSize) const override {
    return intermediateState(source, target, minStepSize);
  }
};

string vec2scad(Vector2f v) {
  ostringstream o;
  o << "[" << v.x() << ", " << v.y() << "]";
  return o.str();
}

typedef array<Vector2f, 2> Line;

string line2scad(Line l) {
  ostringstream o;
  o << "[" << vec2scad(l[0]) << ", " << vec2scad(l[1]) << "]";
  return o.str();
}

void collectTreeLines(const Tree<Vector2f> &rrt, vector<Line> *linesOut) {
  for (const Node<Vector2f> *node : rrt.allNodes()) {
    if (node->parent()) {
      linesOut->push_back({node->state(), node->parent()->state()});
    }
  }
}

template <class T>
void scad_array(fstream *out, string name, const vector<T> &items,
                function<string(T)> printer) {
  *out << name << " = [" << endl;
  for (const T &x : items) {
    *out << "  " << printer(x) << "," << endl;
  }
  *out << "];" << endl;
}

int main(int argc, char **argv) {
  Vector2f start = {0.4 * meter, 0.3 * meter};
  Vector2f goal = {2 * meter, 3 * meter};

  // setup rrt
  auto stateSpace = make_shared<RobotStateSpace>();
  BiRRT<Eigen::Vector2f> biRRT(stateSpace);
  biRRT.setStepSize(0.15 * meter);
  biRRT.setStartState(start);
  biRRT.setGoalState(goal);
  biRRT.setGoalBias(0.1);
  biRRT.setMaxIterations(500);
  biRRT.setGoalMaxDist(0.05 * meter);

  // by default, use a random seed
  unsigned int seed = time(NULL);

  // override using a known seed for repeatable results
  // note: comment this out to use the random seed
  // seed = 1484970367;
  seed = 1484997019;

  srand(seed);
  cout << "INFO: using random seed: " << seed << endl;

  // robots placed to challenge the rrt
  vector<Vector2f> otherRobots = {
      {0.5 * meter, 1.2 * meter}, {1 * meter, 0.9 * meter},
      {1.5 * meter, 1.2 * meter}, {1.3 * meter, 2 * meter},
      {2.2 * meter, 2.1 * meter},
  };

  // mark them as obstacles for the rrt
  stateSpace->robots = otherRobots;

  // run rrt
  bool success = biRRT.run();
  if (!success) {
    cout << "RRT Error: No solution found" << endl;
    return 1;
  }

  // create openscad file
  fstream scadfile;
  string filename = "out.scad";
  scadfile.open(filename, std::fstream::out);

  // export constants to openscad
  auto export_constant = [&scadfile](string name, float value) {
    scadfile << name << " = " << value << ";" << endl;
  };
  export_constant("RobotRadius", RobotRadius);
  export_constant("RobotHeight", RobotHeight);
  export_constant("FieldLength", FieldLength);
  export_constant("FieldWidth", FieldWidth);
  export_constant("RRT_RandomSeed", seed);
  export_constant("meter", meter);

  // Start and goal
  scadfile << "StartPos = " << vec2scad(start) << ";" << endl;
  scadfile << "GoalPos = " << vec2scad(goal) << ";" << endl;

  // other robots
  scad_array<Vector2f>(&scadfile, "ObstacleRobots", otherRobots, vec2scad);

  // RRT Solution
  vector<Vector2f> solution = biRRT.getPath();
  scad_array<Vector2f>(&scadfile, "RRT_Solution", solution, vec2scad);

  // RRT Lines
  vector<Line> treeLines;
  collectTreeLines(biRRT.startTree(), &treeLines);
  collectTreeLines(biRRT.goalTree(), &treeLines);
  scad_array<Line>(&scadfile, "RRT_Lines", treeLines, line2scad);

  // RRT Nodes
  vector<Vector2f> nodes;
  auto s = biRRT.startTree().allNodes();
  auto g = biRRT.goalTree().allNodes();
  transform(s.begin(), s.end(), back_inserter(nodes),
            [](Node<Vector2f> *n) { return n->state(); });
  transform(g.begin(), g.end(), back_inserter(nodes),
            [](Node<Vector2f> *n) { return n->state(); });
  scad_array<Vector2f>(&scadfile, "RRT_Nodes", nodes, vec2scad);

  scadfile.close();

  // done!
  cout << "wrote to file: " << filename << endl;
  return 0;
}
