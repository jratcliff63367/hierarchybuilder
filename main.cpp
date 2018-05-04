#include "HierarchyBuilder.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// This is a small demonstration application that shows how to use the HierarchyBuilder
// class to automatically derive hierarchies of connected rigid bodies and display the results

#ifdef _MSC_VER
#pragma warning(disable:4100)
#endif

// define a randomized list of rigid bodies
const char *gRigidBodyNames[] =
{
	"shoulder_lift_link",
	"box1",
	"torso_lift_link",
	"wrist_flex_link",
	"torso_fixed_link",
	"estop_link",
	"bellows_link",
	"sphere1",
	"bellows_link2",
	"head_tilt_link",
	"sphere3",
	"head_pan_link",
	"box2",
	"l_gripper_finger_link",
	"base_link",
	"r_gripper_finger_link",
	"sphere0",
	"forearm_roll_link",
	"rbody8",
	"laser_link",
	"box10",
	"wrist_roll_link",
	"box3",
	"gripper_link",
	"elbow_flex_link",
	"l_wheel_link",
	"box0",
	"shoulder_pan_link",
	"rbody0",
	"upperarm_roll_link",
	"r_wheel_link",
	"sphere2",
};

class JointRef
{
public:
	const char *jointName;
	const char *body0;
	const char *body1;
};

// Define a list of joints referencing some of the rigid bodies, but not all.
// Joints are defined in random order; hierarchy is implied via the connections and
// must derived.
JointRef gJoints[]
{
	{ "wrist_roll_joint", "wrist_flex_link", "wrist_roll_link" },
	{ "head_pan_joint", "torso_lift_link", "head_pan_link" },
	{ "torso_lift_joint", "base_link", "torso_lift_link" },
	{ "shoulder_pan_joint", "torso_lift_link", "shoulder_pan_link" },
	{ "sphere2-sphere3", "sphere2", "sphere3"},
	{ "l_gripper_finger_joint", "gripper_link", "l_gripper_finger_link" },
	{ "upperarm_roll_joint", "shoulder_lift_link", "upperarm_roll_link" },
	{ "elbow_flex_joint", "upperarm_roll_link", "elbow_flex_link" },
	{ "l_wheel_joint", "base_link", "l_wheel_link" },
	{ "sphere1-sphere2", "sphere1", "sphere2"},
	{ "bellows_joint", "torso_lift_link", "bellows_link" },
	{ "head_camera_depth_joint", "head_camera_link", "head_camera_depth_frame" },
	{ "head_camera_rgb_joint", "head_camera_link", "head_camera_rgb_frame" },
	{ "head_camera_depth_optical_joint", "head_camera_depth_frame", "head_camera_depth_optical_frame" },
	{ "sphere3-sphere1", "sphere3", "sphere1"},
	{ "forearm_roll_joint", "elbow_flex_link", "forearm_roll_link" },
	{ "box2-box3", "box2", "box3" },
	{ "torso_fixed_joint", "base_link", "torso_fixed_link" },
	{ "shoulder_lift_joint", "shoulder_pan_link", "shoulder_lift_link" },
	{ "sphere3-sphere4", "sphere3", "sphere4"},
	{ "r_gripper_finger_joint", "gripper_link", "r_gripper_finger_link" },
	{ "head_tilt_joint", "head_pan_link", "head_tilt_link" },
	{ "bellows_joint2", "torso_lift_link", "bellows_link2" },
	{ "r_wheel_joint", "base_link", "r_wheel_link" },
	{ "gripper_axis", "wrist_roll_link", "gripper_link" },
	{ "estop_joint", "base_link", "estop_link" },
	{ "box1-box2", "box1", "box2"},
	{ "head_camera_joint", "head_tilt_link", "head_camera_link" },
	{ "laser_joint", "base_link", "laser_link" },
	{ "head_camera_rgb_optical_joint", "head_camera_rgb_frame", "head_camera_rgb_optical_frame" },
	{ "wrist_flex_joint", "forearm_roll_link", "wrist_flex_link" },
};

void showHierarchy(const HIERARCHY_BUILDER::HierarchyLink *link, uint32_t depth)
{
	// Display the joints
	uint32_t count = link->getChildCount();
	for (uint32_t i = 0; i < count; i++)
	{
		const char *body0;
		const char *body1;
		bool isLoopJoint;
		const char *j = link->getJoint(i, body0, body1, isLoopJoint);
		for (uint32_t k = 0; k < depth; k++)
		{
			printf("    ");
		}
		printf("Joint(%s) : body0(%s)->body1(%s) : loop(%s)\r\n", j, body0, body1, isLoopJoint ? "true" : "false");
	}
	// Recursively traverse and display the hierarchy of each child node
	for (uint32_t i = 0; i < count; i++)
	{
		const HIERARCHY_BUILDER::HierarchyLink *child = link->getChild(i);
		showHierarchy(child, depth + 1);
	}
}

// Simple demonstration of how to use the HierarchyBuilder class
int main(int argc,const char **argv)
{
	// Create an instance of the hierarchy builder class
	auto hb = HIERARCHY_BUILDER::HierarchyBuilder::create();
	if (hb)
	{
		// Add the rigid body names
		for (auto &i : gRigidBodyNames)
		{
			hb->addRigidBody(i);
		}
		for (auto &i : gJoints)
		{
			hb->addJoint(i.jointName, i.body0, i.body1);
		}
		hb->build(); // derive the hierarchy
		// Print the results
		uint32_t count = hb->getDisconnectedRigidBodyCount();
		printf("DisconnectedRigidBodyCount: %d\r\n", count);
		for (uint32_t i = 0; i < count; i++)
		{
			printf("    RigidBody[%d]=%s\r\n", i, hb->getDisconnectedRigidBody(i));
		}
		uint32_t hcount = hb->getHierarchyCount();
		printf("Found %d hierarchies\r\n", hcount);
		for (uint32_t i = 0; i < hcount; i++)
		{
			printf("========================================================\r\n");
			printf("Hierarchy[%d]\r\n", i);
			printf("========================================================\r\n");
			const HIERARCHY_BUILDER::HierarchyLink *link = hb->getHierarchyRoot(i);
			showHierarchy(link, 0);
			printf("========================================================\r\n");
			printf("\r\n");
		}
		// release the HierarchyBuilder class
		hb->release();
	}

	return 0;
}
