#pragma once

#include <stdint.h>

// **********************************************************************************************************
// This code snippet takes a collection of bodies (by name) and a collection of joints which connect those
// bodies and produces a set of hierarchies for them.  Bodies not connected by any joints are returned 
// separately.
//
// The use case for this is if you have an utterly randomized set of rigid bodies and joints and need to
// derive from it a set of hierarchies (articulations) from the random input dataset.
//
// If there are multiple hierarchies, it will detect and return them.  If the input joints are in
// completely randomized order, it will detect hierarchy fragments and merge them into complete chains.
// If any of the constraints contain a loop (connects back to itself, then that loop is detected and flagged)
//
// Usage is as follows:
//
// Step #1 : Create an instance of the HierarchyBuilder class
// Step #2 : Add all of the named objects
// Step #3 : Add all of the named joints
// Step #4 : Call the 'build' routine to compute the hierarchies and disconnected rigid bodies
// Step #5 : Query the results.
// Step #6 : Release the builder class when finished using it.
//
// Example usage:
//
//  HIERARCHY_BUILDER::HierarchyBuilder *hb = HIERARCHY_BUILDER::HierarchyBuilder::create();
//  for (auto &i:bodies) hb->addRigidBody(i);
//  for (auto &i:joints) hb->addJoint(i.name,i.body0,i.body1);
//  hb->build();
//
//    (query hierarchies and traverse them)
//    (query disconnected rigid bodies)
//
//  hb->release();
//  
//
// Written by John W. Ratcliff : mailto:jratcliffscarab@gmail.com on May 5, 2018
// 
// Released open source under MIT license.
// If you find this code useful, you can send a tip in bitcoin to the following public key:
// 1MYgkrETMDoWzsdLmWVnCUixbTQFc14FUJ : https://live.blockcypher.com/btc/address/1MYgkrETMDoWzsdLmWVnCUixbTQFc14FUJ/
// **********************************************************************************************************

namespace HIERARCHY_BUILDER
{

// A single link in the hierarchy.  Query the children to get the list of joints
// associated with this link.
// Recurse into each child to traverse the entire hierarchy.
class HierarchyLink
{
public:
	virtual void printChain(uint32_t depth) const = 0;
	// Return the number of children links
	virtual uint32_t getChildCount(void) const = 0;

	// Returns the name of the joint and bodies for this child
	virtual const char *getJoint(uint32_t index,	// Child index
								const char *&body0, // Name of parent body
								const char *&body1, // Name of child body
								bool &isLoopJoint) const = 0; // true if this is a loop joint (refers back to a previous body in the hierarchy)

	// Returns this child hierarchy link
	virtual const HierarchyLink *getChild(uint32_t index) const = 0;
};

class HierarchyBuilder
{
public:
	// Create an instance of the HierarchyBuilder class
	static HierarchyBuilder *create(void);

	virtual void reset(void) = 0;	// reset back to initial state

	// Add a reference to a rigid body by unique id, must be unique.  Returns false if the name already exists.
	virtual bool addRigidBody(const char *id) = 0;	// add a reference to a rigid body by name

	// Add a reference to a named joint and which two rigid bodies it references.
	// While rare it is technically possible that more than one joint could connect the same
	// two rigid bodies.  If the joint name is duplicate, it will return false.
	virtual bool addJoint(const char *jointId,const char *body0,const char *body1) = 0; // add a reference to a joint that connects two rigid bodies

	// Build the hierarchy and return the number of unique hierarchies found
	virtual uint32_t build(void) = 0;

	// Returns the number of rigid bodies which were not connected by any joints
	virtual uint32_t getDisconnectedRigidBodyCount(void) = 0;

	// Returns the name of this disconnected rigid body; null if this index is out of range
	virtual const char * getDisconnectedRigidBody(uint32_t index) = 0;

	// returns the number of hierarchies found
	virtual uint32_t getHierarchyCount(void) const = 0;

	// Return the root link of this hierarchy
	virtual const HierarchyLink * getHierarchyRoot(uint32_t index) const = 0;

	// Debug printf the results
	virtual void debugPrint(void) = 0;

	// Methods to query the inputs to the system

	// Return the number of rigid bodies in the system
	virtual uint32_t getRigidBodyCount(void) = 0;
	// Return the name of a rigid body input
	virtual const char *getRigidBody(uint32_t index) = 0;

	// Return the number of joints in the system
	virtual uint32_t getJointCount(void) = 0;
	// Return the name of a joint input and the names of the bodies it connects
	virtual const char *getJoint(uint32_t index, const char *&body0, const char *&body1) = 0;

	// Release the HierarchyBuilder instance
	virtual void release(void) = 0;
protected:
	virtual ~HierarchyBuilder(void)
	{
	}
};

} // End of the HIERARCHY_BUILDER namespace
