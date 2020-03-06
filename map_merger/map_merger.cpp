#include <sstream>
#include <vector>

#include <pthread.h>
#include <math.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"

#define TB3_0_MASK (1 << 0)
#define TB3_1_MASK (1 << 1)
#define TB3_2_MASK (1 << 2)

#define FREE 0
#define UNKNOWN 1
#define OCCUPIED 2

uint8_t recved_map_flags;
pthread_mutex_t map_flags_lock;

nav_msgs::OccupancyGrid robot_maps[3];

ros::Publisher merged_publisher;

static double get_x(nav_msgs::OccupancyGrid& map,
    int i, int j)
{
  // Assume the origin has Pose of zero (i.e. no rotation)
  return (i * map.info.resolution + map.info.origin.position.x);
}

static double get_y(nav_msgs::OccupancyGrid& map,
    int i, int j)
{
  // Assume the origin has Pose of zero (i.e. no rotation)
  return (j * map.info.resolution + map.info.origin.position.y);
}

static int get_i(nav_msgs::OccupancyGrid& map, double x, double y)
{
  return (int)((x - map.info.origin.position.x) / map.info.resolution);
}

static int get_j(nav_msgs::OccupancyGrid& map, double x, double y)
{
  return (int)((y - map.info.origin.position.y) / map.info.resolution);
}

static bool is_occupied_ind(nav_msgs::OccupancyGrid& map,
    int i, int j)
{
  if (i < 0 || i > map.info.width || j < 0 || j > map.info.height)
    return true;
  return (map.data[j * map.info.width + i] >= 50);
}

static bool is_occupied(nav_msgs::OccupancyGrid& map,
    double x, double y)
{
  int i, j;
  i = get_i(map, x, y);
  j = get_j(map, x, y);
  if (i < 0 || i > map.info.width || j < 0 || j > map.info.height)
    return true;
  return (map.data[j * map.info.width + i] >= 50);
}

static int8_t get_value(nav_msgs::OccupancyGrid map,
    double x, double y)
{
  int i, j;
  i = get_i(map, x, y);
  j = get_j(map, x, y);
  return (map.data[j * map.info.width + i]);
}

static bool in_grid(nav_msgs::OccupancyGrid& map, double x, double y)
{
  return ((get_i(map, x, y) < map.info.width)
      && (get_j(map, x, y) < map.info.height));
}

static void increment_bin(long *bins, int theta_len,
    double theta_res, int radius_len, double radius_res,
    double theta, double radius)
{
  int theta_bin = (int)(theta / theta_res);
  int radius_bin = (int)(radius / radius_res);

  if (theta_bin < 0 || theta_bin > theta_len ||
      radius_bin < 0 || radius_bin > radius_len)
    return;

  (bins[theta_bin * radius_len + radius_bin])++;
}

static double find_radius(nav_msgs::OccupancyGrid& map,
    double theta, double x, double y)
{
  return (x * cos(theta) + y * sin(theta));
}

void hough(nav_msgs::OccupancyGrid& map,
    long *bins, uint32_t bins_len,
    int theta_len, double theta_res,
    int radius_len, double radius_res)
{
  double x, y;
  double theta, radius;
  double radius_max = radius_len * radius_res;
  for (int j = 0; j < map.info.height; j++)
  {
    for (int i = 0; i < map.info.width; i++)
    {
      if (is_occupied_ind(map, i, j))
      {
        theta = 0.0;
        for (int k = 0; k < theta_len; k++)
        {
          x = get_x(map, i, j);
          y = get_y(map, i, j);
          radius = find_radius(map, theta, x, y);

          if (radius < radius_max)
          {
            increment_bin(bins, theta_len, theta_res,
                radius_len, radius_res, theta, radius);
          }
          theta += theta_res;
        }
      }
    }
  }
    
}

struct line
{
  double theta;
  double radius;
};

void hough_lines(std::vector<line>& lines,
    nav_msgs::OccupancyGrid& map)
{
  double theta_res = 0.25;
  int theta_len = (int)(M_PI / theta_res);
  double radius_res = map.info.resolution;
  int radius_len = map.info.width * 2;
  long *bins = (long *)calloc(radius_len*theta_len, sizeof(long));
  line ln;

  hough(robot_maps[0], bins, radius_len * theta_len,
      theta_len, theta_res,
      radius_len, radius_res);

  //for (int i = 0; i < radius_len * theta_len; i++)
  //  printf("%ld, ", bins[i]);

  //printf("\n");

  for (int i = 0; i < theta_len; i++)
  {
    for (int j = 0; j < radius_len; j++)
    {
      if (bins[i * radius_len + j] >= 10)
      {
        ln.theta = i * theta_res;
        ln.radius = j * radius_res;
        lines.push_back(ln);
      }
    }
  }

  free(bins);
}

struct line_seg
{
  double theta;
  double radius;
  double start;
  double len;
};

void line_segs(std::vector<line_seg>& line_seg_vec,
    nav_msgs::OccupancyGrid& map)
{
  std::vector<line> lines;
  double x, y;
  double incr_x, incr_y;
  double step = map.info.resolution / 2.0;
  double dist_from_rad;
  double min_dist, max_dist;
  double current_x, current_y;
  double line_seg_len;
  line_seg seg;

  hough_lines(lines, map);

  for (int i = 0; i < lines.size(); i++)
  {
    // Detect line segments
    // x, y are coords of where line radius hits line
    x = lines[i].radius * cos(lines[i].theta);
    y = lines[i].radius * sin(lines[i].theta);
    // Incrs form a unit vector along the line
    incr_x = sin(lines[i].theta);
    incr_y = -cos(lines[i].theta);
    // Dist here is the distance along the line from
    // (x, y) above
    dist_from_rad = 0.0;
    min_dist = 65536.0; // A big number TODO better
    max_dist = -65536.0; // A small number TODO better

    // TODO what if line rad is outside grid??
    current_x = x;
    current_y = y;

    // March along line in backwards direction
    while (in_grid(map, current_x, current_y))
    {
      current_x = x + dist_from_rad*incr_x;
      current_y = y + dist_from_rad*incr_y;
      if (is_occupied(map, current_x, current_y))
      {
        if (dist_from_rad < min_dist)
        {
          min_dist = dist_from_rad;
        }
        if (dist_from_rad > max_dist)
        {
          max_dist = dist_from_rad;
        }
      }
      dist_from_rad -= step;
    }

    // Reset to where radius hits the line
    dist_from_rad = 0.0;
    current_x = x;
    current_y = y;

    // March along line in forwards direction
    while (in_grid(map, current_x, current_y))
    {
      current_x = x + dist_from_rad*incr_x;
      current_y = y + dist_from_rad*incr_y;
      if (is_occupied(map, current_x, current_y))
      {
        if (dist_from_rad < min_dist)
        {
          min_dist = dist_from_rad;
        }
        if (dist_from_rad > max_dist)
        {
          max_dist = dist_from_rad;
        }
      }
      // Different from previous loop
      dist_from_rad += step;
    }
    line_seg_len = max_dist - min_dist;

    if (line_seg_len > 0)
    {
      // Create line_seg
      seg.theta = lines[i].theta;
      seg.radius = lines[i].radius;
      seg.start = min_dist;
      seg.len = line_seg_len;
      line_seg_vec.push_back(seg);
    }
  }
}

//TODO publish a line to the topic
//void publish_line()

/* Represents a rotation by theta, followed by
 * a translation by [x, y]
 */

struct grid_transform
{
  double theta;
  double x;
  double y;
};

bool compare_theta(grid_transform g1, grid_transform g2)
{
  return (g1.theta < g2.theta);
}

double score_transform(nav_msgs::OccupancyGrid& map0, nav_msgs::OccupancyGrid& map1, grid_transform transform)
{
  double correct = 0.0, incorrect = 0.0;
  double score;
  bool map0_occ, map1_occ;
  double map1_tr_x, map1_tr_y;
  for (int j = 0; j < map1.info.height; j++)
  {
    for (int i = 0; i < map1.info.width; i++)
    {
      map1_occ = is_occupied_ind(map1, i, j);
      // Transform the (x, y) by map1's transform
      map1_tr_x = (get_x(map1, i, j) * cos(transform.theta)
          - get_y(map1, i, j) * sin(transform.theta)) + transform.x;
      map1_tr_y = (get_x(map1, i, j) * sin(transform.theta)
          + get_y(map1, i, j) * cos(transform.theta)) + transform.y;

      // Only count the value if it's in the maps' overlapping portion
      if (in_grid(map0, map1_tr_x, map1_tr_y))
      {
        map0_occ = is_occupied_ind(map0, get_i(map0, map1_tr_x, map1_tr_y),
            get_j(map0, map1_tr_x, map1_tr_y));
        if (map0_occ == map1_occ)
        {
          correct += 1.0;
        }
        else
        {
          incorrect += 1.0;
        }
      }
    }
  }

  score = (correct / (correct + incorrect));

  return score;
}

double max_val(std::vector<double>& values)
{
  double max = -65536.0;
  for (int i = 0; i < values.size(); i++)
  {
    if (values[i] > max)
      max = values[i];
  }
  return max;
}

double min_val(std::vector<double>& values)
{
  double min = 65536.0;
  for (int i = 0; i < values.size(); i++)
  {
    if (values[i] < min)
      min = values[i];
  } 
  return min;
}

double transform_x(double x, double y, grid_transform transform)
{
  return x * cos(transform.theta) - y * sin(transform.theta)
      + transform.x;
}

double transform_y(double x, double y, grid_transform transform)
{
  return y * cos(transform.theta) + x * sin(transform.theta)
      + transform.y;
}

double reverse_transform_x(double x, double y,
    grid_transform transform)
{
  return (x - transform.x) * cos(-transform.theta)
      - (y - transform.y) * sin(-transform.theta);
}

double reverse_transform_y(double x, double y,
    grid_transform transform)
{
  return (y - transform.y) * cos(-transform.theta)
      + (x - transform.x) * sin(-transform.theta);
}

nav_msgs::OccupancyGrid merge_maps_transformed(
    nav_msgs::OccupancyGrid& map0,
    nav_msgs::OccupancyGrid& map1,
    grid_transform transform)
{
  int map_width, map_height;
  double min_x, max_x, min_y, max_y;
  nav_msgs::OccupancyGrid merged;
  double map0_x, map0_y, map1_x, map1_y;

  // Calculate bounding box for the merged maps
  std::vector<double> xs, ys;
  // map0 x extremes
  xs.push_back(get_x(map0, 0, 0));
  xs.push_back(get_x(map0, map0.info.width-1, map0.info.height-1));

  // Corners of map1 transformed
  xs.push_back(transform_x(
      get_x(map1, 0, 0), get_y(map1, 0, 0), transform));
  xs.push_back(transform_x(
      get_x(map1, 0, map1.info.height-1),
      get_y(map1, 0, map1.info.height-1),
      transform));
  xs.push_back(transform_x(
      get_x(map1, map1.info.width-1, 0),
      get_y(map1, map1.info.width-1, 0),
      transform));
  xs.push_back(transform_x(
      get_x(map1, map1.info.width-1, map1.info.height-1),
      get_y(map1, map1.info.width-1, map1.info.height-1),
      transform));

  // map0 y extremes
  ys.push_back(get_y(map0, 0, 0));
  ys.push_back(get_y(map0, map0.info.width-1, map0.info.height-1));

  // Corners of map1 transformed
  ys.push_back(transform_y(
      get_x(map1, 0, 0), get_y(map1, 0, 0), transform));
  ys.push_back(transform_y(
      get_x(map1, 0, map1.info.height-1),
      get_y(map1, 0, map1.info.height-1),
      transform));
  ys.push_back(transform_y(
      get_x(map1, map1.info.width-1, 0),
      get_y(map1, map1.info.width-1, 0),
      transform));
  ys.push_back(transform_y(
      get_x(map1, map1.info.width-1, map1.info.height-1),
      get_y(map1, map1.info.width-1, map1.info.height-1),
      transform));

  // Get min and max values
  min_x = min_val(xs);
  max_x = max_val(xs);
  min_y = min_val(ys);
  max_y = max_val(ys);

  // Copy resolution
  merged.info.resolution = map0.info.resolution;

  // Calculate required dimensions
  map_width = (int)ceil((max_x - min_x) / merged.info.resolution);
  map_height = (int)ceil((max_y - min_y) / merged.info.resolution);

  // Calculate origin
  merged.info.origin.position.x = merged.info.resolution
      * floor(min_x / merged.info.resolution);
  merged.info.origin.position.y = merged.info.resolution
      * floor(min_y / merged.info.resolution);

  merged.info.height = map_height;
  merged.info.width = map_width;
  merged.data = std::vector<int8_t>(
      merged.info.width * merged.info.height);

  for (int i = 0; i < merged.data.size(); i++)
  {
    // -1 (unknown) by default
    merged.data[i] = -1;
  }

  for (int j = 0; j < merged.info.height; j++)
  {
    for (int i = 0; i < merged.info.width; i++)
    {
      map0_x = get_x(merged, i, j);
      map0_y = get_y(merged, i, j);
      if (in_grid(map0, map0_x, map0_y)
          && (get_value(map0, map0_x, map0_y) != -1))
      {
          merged.data[j*merged.info.width + i] =
              get_value(map0, map0_x, map0_y);
      }
      else
      {
        map1_x = reverse_transform_x(get_x(merged, i, j),
            get_y(merged, i, j), transform);
        map1_y = reverse_transform_y(get_x(merged, i, j),
            get_y(merged, i, j), transform);
        if (in_grid(map1, map1_x, map1_y))
        {
          merged.data[j*merged.info.width + i] =
              get_value(map1, map1_x, map1_y);
        }
      }
    }
  }

  return merged;
}

// Find a transform of map1 that transforms it onto map0
grid_transform find_transform(nav_msgs::OccupancyGrid map0,
    nav_msgs::OccupancyGrid map1)
{
  // Represent a grid transform
  std::vector<grid_transform> possible_transforms;
  grid_transform transform;

  //std::vector<line> lines[2];
  double line0_unit_x;
  double line0_unit_y;
  std::vector<line_seg> lines[2];

  double slide_dist;

  // Do Hough transform for lines
  //hough_lines(lines[0], robot_maps[0]);
  //hough_lines(lines[1], robot_maps[1]);

  line_segs(lines[0], map0);
  line_segs(lines[1], map1);
  //lines_0.publish()

#if 0
  for (int i = 0; i < lines[0].size(); i++)
  {
    printf("(%lf, %lf)\n", lines[0][i].theta / M_PI,
        lines[0][i].radius);
  }
#endif

  // For each line, we check it against every line in the
  // other map and find transformations that would  match
  // them up
  for (int i = 0; i < lines[0].size(); i++)
  {
    for (int j = 0; j < lines[1].size(); j++)
    {
      line0_unit_x = sin(lines[0][i].theta);  
      line0_unit_y = -cos(lines[0][i].theta);
      // For transform, need the angle difference...
      transform.theta = lines[0][i].theta - lines[1][i].theta;
      // ...and the translation to perform once rotated.
      transform.x = (lines[0][i].radius - lines[1][i].radius)
          * cos(lines[0][i].theta);
      transform.y = (lines[0][i].radius - lines[1][i].radius)
          * sin(lines[0][i].theta);

      // Slide the segment along the line to centre-match them
      slide_dist = (lines[0][i].start
          + 0.5*(lines[0][i].len - lines[1][i].len))
          - lines[1][i].start;
      transform.x += slide_dist * line0_unit_x;
      transform.y += slide_dist * line0_unit_y;

      // Add it to the heap
      possible_transforms.push_back(transform);

      // For every line, there are 2 possible transforms
      // - one is pi radians out from the other.
      transform.theta = lines[0][i].theta
          - lines[1][i].theta
          + M_PI;
      // The second transform is the other way round so the 
      // translation is reversed relative to the non-transformed
      // line. This means the radii are added not subtracted.
      transform.x = (lines[0][i].radius + lines[1][i].radius)
          * cos(lines[0][i].theta);
      transform.y = (lines[0][i].radius + lines[1][i].radius)
          * sin(lines[0][i].theta);

      // Slide the segment along the line to centre-match them
      slide_dist = (lines[0][i].start +
          + 0.5*(lines[0][i].len - lines[1][i].len))
          + lines[1][i].start + lines[1][i].len;
      transform.x += slide_dist * line0_unit_x;
      transform.y += slide_dist * line0_unit_y;
      
      possible_transforms.push_back(transform);
    }
  }

  // Get angles into range 0 <= theta < 2*pi
  for (int i = 0; i < possible_transforms.size(); i++)
  {
    while (possible_transforms[i].theta < 0)
      possible_transforms[i].theta += 2.0*M_PI;
    while (possible_transforms[i].theta >= 2.0*M_PI)
      possible_transforms[i].theta -= 2.0*M_PI;
  }

  // Remove duplicates (~) from possible_transforms
  std::vector<grid_transform> deduped;
  bool is_dupl;
  for (int i = 0; i < possible_transforms.size(); i++)
  {
    is_dupl = false;
    for (int ind = 0; ind < deduped.size(); ind++)
    {
      if ((abs(deduped[ind].x - possible_transforms[i].x) < 0.0625)
          && (abs(deduped[ind].y - possible_transforms[i].y)
          < 0.0625)
          && (abs(deduped[ind].theta - possible_transforms[i].theta)
           < 0.0625))
      {
        is_dupl = true;
      }
    }
    if (!is_dupl)
    {
      deduped.push_back(possible_transforms[i]);
    }
  }

  possible_transforms = deduped;
  for (int i = 0; i < possible_transforms.size(); i++)
  {
      printf("(%lf, %lf, %lf)\n", possible_transforms[i].theta,
          possible_transforms[i].x, possible_transforms[i].y);
  }

  double max_score = 0.0, score = 0.0;
  int max_ind = 0;
  for (int ind = 0; ind < possible_transforms.size(); ind++)
  {
    score = score_transform(robot_maps[0], robot_maps[1], possible_transforms[ind]);
    if (score > max_score)
    {
      max_ind = ind;
      max_score = score;
    }
    printf("ind: %d, score: %lf\n", ind, score);
  }

  printf("Max score : (ind: %d, score: %lf)\n", max_ind, max_score);
  return possible_transforms[max_ind];
}

void merge_maps_and_publish(ros::Publisher p)
{
  ROS_INFO("MERGING...\n");

  grid_transform transform =
      find_transform(robot_maps[0], robot_maps[1]);
  // TODO transform highest scoring map and build new map

  nav_msgs::OccupancyGrid merged_map =
      merge_maps_transformed(robot_maps[0],
          robot_maps[1],
          transform);

  transform = find_transform(merged_map, robot_maps[2]);

  nav_msgs::OccupancyGrid merged_map2 =
      merge_maps_transformed(merged_map,
          robot_maps[2],
          transform);

  p.publish(merged_map2);

}

void tb3_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map,
    uint8_t tb3_no)
{
  pthread_mutex_lock(&map_flags_lock);
  // Copy to map array
  robot_maps[tb3_no] = *map;
  recved_map_flags |= (1 << tb3_no);
  if (recved_map_flags == (TB3_0_MASK | TB3_1_MASK | TB3_2_MASK))
  {
    // Bit heavyweight - TODO don't call directly (but much sync)
    merge_maps_and_publish(merged_publisher);
    // Reset the flag bits
    recved_map_flags = 0;
  }
  pthread_mutex_unlock(&map_flags_lock);
}

void tb3_0_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  tb3_map_callback(map, 0);
}

void tb3_1_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  tb3_map_callback(map, 1);
}

void tb3_2_map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  tb3_map_callback(map, 2);
}

int main(int argc, char **argv)
{
  pthread_mutex_init(&map_flags_lock, 0);

  ros::init(argc, argv, "map_merger");

  ros::NodeHandle n;
  merged_publisher = n.advertise<nav_msgs::OccupancyGrid>("merged_map", 20);

  ros::Rate pub_rate(10);
  int count = 0;
  std_msgs::String msg;
  std::stringstream ss;

  nav_msgs::OccupancyGrid merged_map;
  merged_map.info.width = 5;
  merged_map.info.height = 5;
  merged_map.info.resolution = 0.2;
  merged_map.data = std::vector<int8_t>(25);

  merged_map.info.origin.position.x = -2.;
  merged_map.info.origin.position.y = -2.;

  ros::Subscriber sub_tb3_0_map = n.subscribe("/tb3_0/map", 20,
      tb3_0_map_callback);
  ros::Subscriber sub_tb3_1_map = n.subscribe("/tb3_1/map", 20,
      tb3_1_map_callback);
  ros::Subscriber sub_tb3_2_map = n.subscribe("/tb3_2/map", 20,
      tb3_2_map_callback);

  for (int i = 0; i < 25; i++)
  {
    merged_map.data[i] = 50;
  }

  while (ros::ok())
  {
    ss << "Msg no " << count << "\n";
    msg.data = ss.str();

    //ROS_INFO("%s", msg.data.c_str());

    //pub.publish(merged_map);

    ros::spinOnce();

    pub_rate.sleep();
    count++;
  }

  pthread_mutex_destroy(&map_flags_lock);
  return 0;
}

