#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <stdlib.h>
#include <cmath>
#include "tf/tf.h"
#include <geometry_msgs/Quaternion.h>
#include <vector>
#include "jackal_nodes/PInfoTime.h"
#include "jackal_nodes/PotPInfoTime.h"

using namespace std;
float x_pos;
float y_pos;
float xd;
float yd;
float yaw;
float lin_x = -1.0;
float ang_actual = -1.0;
float ang_z = -1.0;
float new_x = -1.0;
float new_y = -1.0;
float new_yaw = -1.0;
int col = -1;

struct p_pos
{
	float x;
	float y;
	float yaw;
};

struct control_input
{
	float lin_x;
	float ang_z;
};

struct particle_info
{
	vector<p_pos> pos;
	vector<control_input> in;
	float energy;
	int col_num;
	int num;
};

struct potential_particle_info
{
	p_pos pos;
	control_input in;
	float weight;
	int collision;
};

//TODO: CHANGE THIS AND PYTHON OBSTACLE PUBLISHER BASED ON DIFFERENT SCENARIOS
//TODO: CHANGE IF NEEDED, ang[4] is largest ang_z
const int obs_len = 1;
vector<bool> obs_pos(obs_len);
vector<bool> obs_vel(obs_len);
ros::Publisher particle_planner;
ros::Publisher pot_particle_planner;


vector<float> obs_x_pos(obs_len);
vector<float> obs_y_pos(obs_len);
vector<float> obs_x_vel(obs_len);
vector<float> obs_y_vel(obs_len);
vector<float> obs_x_acc(obs_len);
vector<float> obs_y_acc(obs_len);

/*Highway*/
//float oxv [12] = {0.25, -0.25, 0.25, -0.25, 0.25, -0.25, 0.25, -0.25, 0.25, -0.25, 0.25, -0.25};
//float oyv [12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//float oxv [6] = {0.25, -0.25, 0.25, -0.25, 0.25, -0.25};
//float oyv [6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

/*Raceway
float oxv [12] = {0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.4};
float oyv [12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//float oxv [6] = {0.2, 0.4, 0.2, 0.4, 0.2, 0.4};
//float oyv [6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};*/


float dt = 0.1; //loop rate = 10
float planning_dt = 0.5;
int dt_multiplier = 5;
int num_timesteps_planning = 10;//50;//35;
int num_timesteps_action = num_timesteps_planning;
uint num_particles = 200;
vector<particle_info> particles_path;
vector<potential_particle_info> particles_path_temp;

//jackal_nodes::PInfoTime viz_particles;
//jackal_nodes::PotPInfoTime viz_pot_particles;

float max_dist = 60;
float start_x = -40;

float sensing_radius = 10000000.0;
float rail_y = 0.8;//1.8 for wide highway;

vector<float> enhanced_obs_x_pos;
vector<float> enhanced_obs_y_pos;

float buffer = 0.8;


const int num_combinations = 5;
float lin_x_options [num_combinations] = {0, 0.25, 2.0, 1.0, 1.0};
float lin_vel_options [num_combinations] = {0, 0.25, 2.0, 1.0, 1.0};
float ang_z_options [num_combinations] = {0, 0, 0, -1.0, 1.0};
float ang_vel_options [num_combinations] = {0, 0, 0, -0.62, 0.62};

float en_multiplier = 0.0;
float min_w = ang_z_options[4]*en_multiplier*-1; //TODO: CHANGE IF NEEDED, ang[4] is largest ang_z

void odomCallback(const nav_msgs::Odometry::ConstPtr& data)
{
  yaw = tf::getYaw(data->pose.pose.orientation);  
}
void obsCallback(const geometry_msgs::PoseArray::ConstPtr& data)
{
	//ROS_INFO("obsCallback");
	//enhanced_obs_x_pos.clear();
	//enhanced_obs_y_pos.clear();
	//ROS_INFO("obsCallback AAAA");
	if(data != NULL && &(data->poses) != NULL){
		for(int i = 0; i<obs_len;i++) //TODO: replace with data->poses length?
		{
			//ROS_INFO("obsCallback a");
			float o_x = data->poses[i].position.x;
			float o_y = data->poses[i].position.y;
			float dfo = sqrt(pow(o_x-x_pos,2)+pow(o_y-y_pos,2));
			//ROS_INFO("X Obs callback: %f, Y Obs callback: %f", o_x, o_y);
			//ROS_INFO("obsCallback A");
			if(dfo>sensing_radius)
			{
				//ROS_INFO("obsCallback B");
				obs_pos[i] == false;
				obs_vel[i] == false;
				obs_x_acc[i] = 0.0;
				obs_y_acc[i] = 0.0;
				obs_x_vel[i] = 0.0;
				obs_y_vel[i] = 0.0;
				obs_x_pos[i] = 0.0;
				obs_y_pos[i] = 0.0;
			}
			else
			{
				//ROS_INFO("obsCallback C");
				if(obs_pos[i] == false) // Only update position
				{
					//ROS_INFO("obsCallback D");
					obs_x_pos[i] = o_x;
					obs_y_pos[i] = o_y;
					obs_pos[i] = true;
				}
				else //update veloctiy
				{
					//ROS_INFO("obsCallback E");
					float v_x = (o_x-obs_x_pos[i])/dt;
					float v_y = (o_y-obs_y_pos[i])/dt;
					if(obs_vel[i] == false) //update velocity position only
					{
						//ROS_INFO("obsCallback F");
						obs_x_vel[i] = v_x;
						obs_y_vel[i] = v_y;
						obs_x_pos[i] = o_x;
						obs_y_pos[i] = o_y;
						obs_vel[i] = true;
					}
					else // update velocity, position, and acceleration
					{
						//ROS_INFO("obsCallback G");
						float a_x = (v_x-obs_x_vel[i])/dt;
						float a_y = (v_y-obs_y_vel[i])/dt;
						obs_x_acc[i] = a_x;
						obs_y_acc[i] = a_y;
						obs_x_vel[i] = v_x;
						obs_y_vel[i] = v_y;
						obs_x_pos[i] = o_x;
						obs_y_pos[i] = o_y;
					}

				}
			}
		}
		} 
}
/*void obsCallback(const geometry_msgs::PoseArray::ConstPtr& data)
{
	if(data != NULL && &(data->poses) != NULL){
		for(int i = 0; i<obs_len;i++) //TODO: replace with data->poses length?
		{
			//ROS_INFO("obsCallback a");
			float o_x = data->poses[i].position.x;
			float o_y = data->poses[i].position.y;
			float dfo = sqrt(pow(o_x-x_pos,2)+pow(o_y-y_pos,2));
			//ROS_INFO("X Obs callback: %f, Y Obs callback: %f", o_x, o_y);
			//ROS_INFO("obsCallback A");
			if(dfo>sensing_radius)
			{
				//ROS_INFO("obsCallback B");
				obs_pos[i] == false;
				obs_vel[i] == false;
				obs_x_acc[i] = 0.0;
				obs_y_acc[i] = 0.0;
				obs_x_vel[i] = 0.0;
				obs_y_vel[i] = 0.0;
				obs_x_pos[i] = 0.0;
				obs_y_pos[i] = 0.0;
			}
			else
			{
				//ROS_INFO("obsCallback D");
				obs_x_pos[i] = o_x;
				obs_y_pos[i] = o_y;
				obs_pos[i] = true;
				obs_vel[i] = true;
				obs_x_acc[i] = 0.0;
				obs_y_acc[i] = 0.0;
				obs_x_vel[i] = oxv[i];
				obs_y_vel[i] = oyv[i];
			}
		}
		} 
}*/
void gpsCallback(const geometry_msgs::Pose::ConstPtr& data)
{
	x_pos = data->position.x;
	y_pos = data->position.y;
}
void goalCallback(const geometry_msgs::Pose::ConstPtr& data)
{
	xd = data->position.x; // Use for Competition
	yd = data->position.y; // Use for Competition
}
float getAngleDiff(float a1, float a2) // (current angle, goal angle)
{
	float angle_err=a2-a1;
	if(abs(a2-a1-2*M_PI) < abs(angle_err))
	{
		angle_err = a2-a1-2*M_PI;
	}
	if(abs(a2-a1+2*M_PI) < abs(angle_err))
	{
		angle_err = a2-a1+2*M_PI;
	}
	return angle_err;
}
float getAngleAdd(float a1, float a2)
{
	float total_angle = a1+a2;
	while(total_angle>M_PI)
	{
		total_angle = total_angle-(2*M_PI);
	}
	while(total_angle<-M_PI)
	{
		total_angle = total_angle+(2*M_PI);
	}
	return total_angle;
}

float validate_particle(int rand_num, float curr_yaw, float curr_x, float curr_y)
{	
	
	ang_actual = ang_vel_options[rand_num];
	ang_z = ang_z_options[rand_num];
	lin_x = lin_x_options[rand_num];
	float lin_actual = lin_vel_options[rand_num];
	new_yaw = curr_yaw;// - (ang*planning_dt);
	new_x = curr_x;// + (lin_actual*planning_dt)*cos(new_yaw);
	new_y = curr_y;// + (lin_actual*planning_dt)*sin(new_yaw);
	float weight = 0.0;
	bool safe = true;
	for(int k =0;k<dt_multiplier;k++)
	{
		new_yaw=getAngleAdd(ang_actual*dt,new_yaw);
		new_x+=(lin_actual*dt)*cos(new_yaw);
		new_y+=(lin_actual*dt)*sin(new_yaw);
	}
	//check validity
	// if valid, update particles[] input (timestep) and particles[] pos (timestep + 1)
	// else, return false
	if(abs(new_y) >= rail_y)
	{
		safe = false;
		weight = rail_y-abs(new_y)+min_w; //nonsafe weights should always be below safe weights

	}
	for(int k =0; k<enhanced_obs_x_pos.size(); k++)
	{
		float obs_dist = sqrt(pow(enhanced_obs_x_pos[k]-new_x,2)+pow(enhanced_obs_y_pos[k]-new_y,2));
		if(obs_dist <= buffer) // TODO: instead of returning false, make weight 0?
		{
			safe = false;
			weight = min(weight,obs_dist-buffer+min_w);
		}
	}
	if(safe)
	{
		weight = (new_x-curr_x)-(abs(ang_z)*en_multiplier);
		col = 0;
	}
	else
	{
		col=1;
	}
	return weight;
}

void publish_particle(int bi) //TODO: make everying in the form of the messages, including generating plan
{
	jackal_nodes::PInfoArray pa;
	vector<jackal_nodes::PInfo> pia;
	for(int i =0; i<particles_path.size(); i++)
	{
		jackal_nodes::PInfo msg_info;
		vector<jackal_nodes::PPos> msg_pos;
		vector<jackal_nodes::ControlInput> msg_ci;
		msg_info.energy = particles_path[i].energy;
		msg_info.colnum = particles_path[i].col_num;
		msg_info.num = particles_path[i].num;
		for(int j = 0;j<particles_path[i].pos.size();j++)
		{
			if(j==(particles_path[i].pos.size()-1))
			{
				jackal_nodes::ControlInput c;
				c.lin_x = particles_path[i].in[j].lin_x;
				c.ang_z = particles_path[i].in[j].ang_z;
				msg_ci.push_back(c);
			}
			jackal_nodes::PPos p;
			p.x = particles_path[i].pos[j].x;
			p.y = particles_path[i].pos[j].y;
			p.yaw = particles_path[i].pos[j].yaw;
			msg_pos.push_back(p);
		}
		msg_info.pos = msg_pos;
		msg_info.in = msg_ci;
		pia.push_back(msg_info);
	}
	pa.parray = pia;
	pa.best_particle = bi;
	particle_planner.publish(pa);
}

//jackal_nodes::PInfoTime viz_particles;
//jackal_nodes::PotPInfoTime viz_pot_particles;


//TODO: Weight particles based on x distance (later on make it racetrack progress) done
int generate_plan() // generates plan, updates particle 2D vectors on positions (x, y, theta), and inputs (lin.x, ang.z)
{
	// initialize particles with current pos, yaw
	jackal_nodes::PInfoTime viz_particles;
	jackal_nodes::PotPInfoTime viz_pot_particles;
	
	particles_path.clear();
	vector<p_pos> pos;
	vector<control_input> in;
	struct particle_info pi = {pos, in, 0.0, 0, num_particles};
	particles_path.push_back(pi);
	struct p_pos p1 = {x_pos, y_pos, yaw};
	particles_path[0].pos.push_back(p1);
	
	jackal_nodes::PInfoArray viz_pa;
	jackal_nodes::PInfo viz_pi;
	jackal_nodes::PPos viz_pos;
	
	viz_pos.x = x_pos;
	viz_pos.y = y_pos;
	viz_pos.yaw = yaw;
	
	viz_pi.pos.push_back(viz_pos);
	viz_pi.energy = 0.0;
	viz_pi.colnum = 0;
	viz_pi.num = num_particles;
	viz_pi.particle_id = 0;
	
	viz_pa.parray.push_back(viz_pi);
	viz_pa.best_particle = 0;
	viz_particles.pt.push_back(viz_pa);
	
	for(int j = 0; j<num_timesteps_planning; j++) // for each time step, update enhanced_pos
	{
		int p_id = 0;
		jackal_nodes::PotPInfoArray viz_pot_pa;
		jackal_nodes::PInfoArray viz_part_pa;
		
		enhanced_obs_x_pos.clear();
		enhanced_obs_y_pos.clear();
		for(int i = 0; i< obs_len; i++) //TODO: can make obstacle vector more efficient. Instead of clearing every time, just delete the older timestep and add new timestep
		{
			if(obs_vel[i]==true)
			{
				enhanced_obs_x_pos.push_back(obs_x_pos[i] + (obs_x_vel[i]*planning_dt*j));
				enhanced_obs_y_pos.push_back(obs_y_pos[i] + (obs_y_vel[i]*planning_dt*j));
				/*enhanced_obs_x_pos.push_back(obs_x_pos[i] + (obs_x_vel[i]*planning_dt*(j+1)));
				enhanced_obs_y_pos.push_back(obs_y_pos[i] + (obs_y_vel[i]*planning_dt*(j+1)));
				enhanced_obs_x_pos.push_back(obs_x_pos[i] + (obs_x_vel[i]*planning_dt*(j+2)));
				enhanced_obs_y_pos.push_back(obs_y_pos[i] + (obs_y_vel[i]*planning_dt*(j+2)));*/
			}
			else
			{
				enhanced_obs_x_pos.push_back(obs_x_pos[i]);
				enhanced_obs_y_pos.push_back(obs_y_pos[i]);
			}
		}
		int particles_path_size = particles_path.size();
		
		for(int i =0; i < particles_path_size; i++) //update current input, and next position of 
		{
			// pop out particles_path from the front 
			particle_info p_pop = particles_path.front();
			particles_path.erase(particles_path.begin());
			//clear temp vector??
			int viz_pot_pa_begin = viz_pot_pa.potparray.size();
			float total_weight = 0.0;
			for(int k = 0; k<num_combinations;k++)// either choose distribute particles by a weighting, or evenly choose best X particles 
			{
				//check if combo is valid.
				float weight = validate_particle(k, p_pop.pos[j].yaw, p_pop.pos[j].x, p_pop.pos[j].y);
				if(weight > 0.0)
				{
					total_weight+=weight;
				}
				//add new particles to temp vector with weighting
				struct p_pos p1 = {new_x, new_y, new_yaw};
				struct control_input c1 = {lin_x, ang_z};
				struct potential_particle_info ppi = {p1, c1, weight, col};
				
				//Create PotPInfo
				jackal_nodes::PotPInfo viz_pot_pi;
				viz_pot_pi.pos.x = new_x;
				viz_pot_pi.pos.y = new_y;
				viz_pot_pi.pos.yaw = new_yaw;
				viz_pot_pi.in.lin_x = lin_x;
				viz_pot_pi.in.ang_z = ang_z;
				viz_pot_pi.weight = weight;
				viz_pot_pi.collision = col;
				viz_pot_pi.num_particles = 0; //Change later
				viz_pot_pi.parent_id = viz_particles.pt[j].parray[i].particle_id;
				 	
				
				//Add PotPinfo in weighted order
				//Order based on weight
				int l =0;
				while(l<particles_path_temp.size() && weight<particles_path_temp[l].weight)
				{
					l++;
				}
				if(l==particles_path_temp.size())
				{
					particles_path_temp.push_back(ppi);
					viz_pot_pa.potparray.push_back(viz_pot_pi);
				}
				else
				{
					particles_path_temp.insert(particles_path_temp.begin()+l, ppi);
					viz_pot_pa.potparray.insert(viz_pot_pa.potparray.begin()+l+viz_pot_pa_begin,viz_pot_pi);
				}
			}
			
			
			// based on weighting, split up particles
			// add back to back of the particles_path back.
			
			int remaining_particles = p_pop.num;
			int particles_path_temp_size = particles_path_temp.size();
			for(int k =0; k < particles_path_temp_size; k++)
			{
				potential_particle_info pot_p_pop = particles_path_temp.front();
				particles_path_temp.erase(particles_path_temp.begin());
				int num_part_assign = 0;
				if(pot_p_pop.weight > 0.0)
				{
					num_part_assign = (int)ceil((float)p_pop.num * (pot_p_pop.weight/total_weight));
				}
				else
				{
					num_part_assign = (int)ceil((float)remaining_particles/2.0);
				}
				
				if(num_part_assign > remaining_particles || k==particles_path_temp_size-1)
				{
					num_part_assign = remaining_particles;
				}
				viz_pot_pa.potparray[k+viz_pot_pa_begin].num_particles = num_part_assign;
				remaining_particles = remaining_particles-num_part_assign;
				if(num_part_assign > 0)
				{
					//create new track and update it
					jackal_nodes::PInfo viz_info;
					
					vector<p_pos> pos;
					vector<control_input> in;
					for(int l = 0; l < p_pop.pos.size(); l++)
					{
						jackal_nodes::PPos viz_p;
						viz_p.x = p_pop.pos[l].x;
						viz_p.y = p_pop.pos[l].y;
						viz_p.yaw = p_pop.pos[l].yaw;
						viz_info.pos.push_back(viz_p);
						
						pos.push_back(p_pop.pos[l]);
						if(l<p_pop.pos.size()-1)
						{
							in.push_back(p_pop.in[l]);
							jackal_nodes::ControlInput viz_i;
							viz_i.lin_x = p_pop.in[l].lin_x;
							viz_i.ang_z = p_pop.in[l].ang_z;
							viz_info.in.push_back(viz_i);
						}
					}
					jackal_nodes::PPos viz_p;
					viz_p.x = pot_p_pop.pos.x;
					viz_p.y = pot_p_pop.pos.y;
					viz_p.yaw = pot_p_pop.pos.yaw;
					viz_info.pos.push_back(viz_p);
					pos.push_back(pot_p_pop.pos);

					jackal_nodes::ControlInput viz_i;
					viz_i.lin_x = pot_p_pop.in.lin_x;
					viz_i.ang_z = pot_p_pop.in.ang_z;
					viz_info.in.push_back(viz_i);
					in.push_back(pot_p_pop.in);

					float en = (abs(pot_p_pop.in.ang_z)*en_multiplier);
					struct particle_info pi = {pos, in, p_pop.energy+en, p_pop.col_num+pot_p_pop.collision, num_part_assign};
					viz_info.energy = p_pop.energy+en;
					viz_info.colnum = p_pop.col_num+pot_p_pop.collision;
					viz_info.num = num_part_assign;
					viz_info.particle_id = p_id;
					p_id++;
					viz_part_pa.parray.push_back(viz_info);
					particles_path.push_back(pi);
				}
			}
		}
		//find best particle in this timestep
		int best_index = -1;
		float best_w = -10000000000.0;
		int best_col = 1000000000;
		//TODO: different ways to weight progress vs energy
	
		for(int i =0; i < particles_path.size(); i++)
		{
			if(particles_path[i].col_num < best_col)
			{
				best_w = (particles_path[i].pos[j+1].x-particles_path[i].energy);
				best_index = i;
				best_col = particles_path[i].col_num;
			}
			else if(particles_path[i].col_num == best_col && (particles_path[i].pos[j+1].x-particles_path[i].energy) > best_w)
			{
				best_w = (particles_path[i].pos[j+1].x-particles_path[i].energy);
				best_index = i;
				best_col = particles_path[i].col_num;
			}

		}		
		viz_part_pa.best_particle = best_index;
		viz_particles.pt.push_back(viz_part_pa);
		viz_pot_particles.potpt.push_back(viz_pot_pa);
	}

	// choose best particle
	int best_index = -1;
	float best_w = -10000000000.0;
	int best_col = 1000000000;
	//TODO: different ways to weight progress vs energy
	
	for(int i =0; i < particles_path.size(); i++)
	{
		if(particles_path[i].col_num < best_col)
		{
			best_w = (particles_path[i].pos[num_timesteps_planning].x-particles_path[i].energy);
			best_index = i;
			best_col = particles_path[i].col_num;
		}
		else if(particles_path[i].col_num == best_col && (particles_path[i].pos[num_timesteps_planning].x-particles_path[i].energy) > best_w)
		{
			best_w = (particles_path[i].pos[num_timesteps_planning].x-particles_path[i].energy);
			best_index = i;
			best_col = particles_path[i].col_num;
		}

	}
	//publish_particle(best_index);
	particle_planner.publish(viz_particles);
	pot_particle_planner.publish(viz_pot_particles);
	return best_index; 
}
int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "particle_planner_highway_race");
	ros::NodeHandle nh;
	ros::Publisher vel_pub0;
  ros::Publisher obsxvel;
  ros::Publisher obsyvel;
	vel_pub0 = nh.advertise<geometry_msgs::Twist>("/jackal0/jackal_velocity_controller/cmd_vel", 1, true);
	particle_planner = nh.advertise<jackal_nodes::PInfoTime>("/particle_info_time", 1, true);
	pot_particle_planner = nh.advertise<jackal_nodes::PotPInfoTime>("/pot_particle_info_time", 1, true);
	obsxvel = nh.advertise<std_msgs::Float32MultiArray>("/obstacle/x_vel", 1, true);
  obsyvel = nh.advertise<std_msgs::Float32MultiArray>("/obstacle/y_vel", 1, true);

	
	ros::Subscriber odomsub = nh.subscribe("/jackal0/odometry/local_filtered", 10, odomCallback);
	ros::Subscriber gpssub = nh.subscribe("/jackal0/global_pos", 10, gpsCallback);
	ros::Subscriber goalsub = nh.subscribe("/jackal0/goal_pos", 10, goalCallback);
	ros::Subscriber obssub = nh.subscribe("/obstacle/global_pos", 10, obsCallback);
	ros::Rate loop_rate(10);
	geometry_msgs::Twist vel0;
	std_msgs::Float32MultiArray xv, yv;
	vector<float> xv_d;
	vector<float> yv_d;
	for(int l = 0;l<obs_len;l++)
	{
		if(obs_vel[l] == false)
		{
			xv_d.push_back(0.0);
			yv_d.push_back(0.0);
		}
		else
		{
			xv_d.push_back(obs_x_vel[l]);
			yv_d.push_back(obs_y_vel[l]);
		}	

	}
	xv.data = xv_d;
	yv.data = yv_d;
	obsxvel.publish(xv);
	obsyvel.publish(yv);
	
	vel0.linear.x = 0;//linear velocity(m/s)
	vel0.angular.z = 0;//angular velocity(rad/s)

	int calibrated = 0;
	float dfg = 10.0;
	float time = 0;
	int abc = -1;
	int particle_index = -100;

	while (ros::ok())
  {
		ROS_INFO("========= ROS LOOP =========");
		vel_pub0.publish(vel0);
		ros::spinOnce();
		loop_rate.sleep();
		time+=dt;
		dfg = sqrt(pow(xd-x_pos,2)+pow(yd-y_pos,2));

		if(calibrated <= 5)
		{
			for(int i = 0;i< obs_len;i++)
			{
				ROS_INFO("Obs %d X: %f, Y: %f", i, obs_x_pos[i], obs_y_pos[i]);
			}
			calibrated++;
		}
		else if(dfg>0.5)
		{
			particle_index = generate_plan();
			if(particle_index==-1)
			{
				ROS_INFO("no plans");
			}
			/*float wayp_x = particles_path[particle_index].pos[1].x;
			float wayp_y = particles_path[particle_index].pos[1].y;
			float dfw = sqrt(pow(wayp_x-x_pos,2)+pow(wayp_y-y_pos,2));
			float thetaw=atan2((wayp_y-y_pos),(wayp_x-x_pos));
			float wayp_yaw = particles_path[particle_index].pos[1].yaw;
			float ang_err = getAngleDiff(yaw, wayp_yaw);
			ROS_INFO("x: %f, y: %f, yaw: %f", x_pos, y_pos, yaw);
			ROS_INFO("wayp_x: %f, wayp_y: %f, wayp_yaw: %f, dfw: %f, ang_err: %f, lin_x: %f, ang_z: %f", wayp_x, wayp_y, wayp_yaw, dfw, ang_err, dfw/0.5, 2.0*ang_err);*/
			vel0.linear.x = particles_path[particle_index].in[0].lin_x; //dfw/0.5;
			vel0.angular.z = particles_path[particle_index].in[0].ang_z; //2.0*ang_err;
			/*for(int i = 0;i<num_timesteps_planning; i++)
			{
				ROS_INFO("timestep: %d, x: %f, y: %f, yaw: %f, lin_x: %f, ang_z: %f", i, particles_path[particle_index].pos[i].x, particles_path[particle_index].pos[i].y, particles_path[particle_index].pos[i].yaw, particles_path[particle_index].in[i].lin_x, particles_path[particle_index].in[i].ang_z);
			}
			ROS_INFO("timestep: %d, x: %f, y: %f, yaw: %f", num_timesteps_planning, particles_path[particle_index].pos[num_timesteps_planning].x, particles_path[particle_index].pos[num_timesteps_planning].y, particles_path[particle_index].pos[num_timesteps_planning].yaw);*/
		}
		else
		{
			vel0.linear.x = 0.0;
			vel0.angular.z = 0.0;
			break;
		}
	}			
	return 0;
}
