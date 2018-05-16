#include "HierarchyBuilder.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <string>
#include <unordered_map>
#include <vector>

#ifdef _MSC_VER
#pragma warning(disable:4100)
#endif

#define LOG_CHAIN 0	// True to debug how the hierarchy chain is being built

namespace HIERARCHY_BUILDER
{

class RigidBodyRef
{
public:
	std::string	mName;
	bool		mUsed{ false };	//whether not this rigid body is part of an hierarchy
};

typedef std::vector< RigidBodyRef > RigidBodyRefVector;
typedef std::vector< std::string > StringVector;

class JointRef
{
public:
	bool		mUsed{ false };	// whether or not this joint has been added to an hierarchy yet or not
	std::string	mName;
	std::string	mBody0;
	std::string mBody1;
};

typedef std::vector< JointRef > JointRefVector;

class Link;

typedef std::vector< Link *> LinkVector;

class Link : public HierarchyLink
{
public:
	Link(void)
	{
	}
	virtual ~Link(void)
	{
		for (auto &i : mChildren)
		{
			delete i;
		}
	}

	enum LinkOrder
	{
		LO_NOT_LINKED,
		LO_BODY0,
		LO_BODY1
	};

	LinkOrder isLinked(const JointRef &jref) const
	{
		LinkOrder lo = LO_NOT_LINKED;

		if (mRigidBody == jref.mBody0)
		{
			lo = LO_BODY0;
		}
		else if (mRigidBody == jref.mBody1)
		{
			lo = LO_BODY1;
		}

		return lo;
	}


	Link * add(const JointRef &jref)
	{
		Link *ret = nullptr;

		LinkOrder lo = isLinked(jref);

		if (lo != LO_NOT_LINKED)
		{
			if (lo == LO_BODY0)
			{
				ret = new Link;
				ret->mJointName = jref.mName;
				ret->mRigidBody = jref.mBody1;
				mChildren.push_back(ret);
			}
			else
			{
				ret = new Link;	
				ret->mJointName = jref.mName;
				ret->mRigidBody = jref.mBody1;
				ret->mChildren = mChildren;	// New link inherits his children
				mChildren.clear();
				mRigidBody = jref.mBody0; // 
				mChildren.push_back(ret); // new head of linked list...
			}
		}
		else
		{
			// Recursively descend the tree to see if this joint attaches to anywhere in the existing
			// hierarchy
			for (auto &i : mChildren)
			{
				ret = i->add(jref);
				if (ret)
				{
					break;
				}
			}
		}

		return ret;
	}

	void indent(uint32_t depth) const
	{
		for (uint32_t i = 0; i < depth; i++)
		{
			printf("    ");
		}
	}

	virtual void printChain(uint32_t depth) const override final
	{
		for (auto &i : mChildren)
		{
			indent(depth);
			printf("%s->%s  : JointName: %s : IsLoopJoint(%s)\r\n", 
				mRigidBody.c_str(), 
				i->mRigidBody.c_str(), 
				i->mJointName.c_str(), 
				i->mIsLoopJoint ? "true" : "false");
		}
		for (auto &i : mChildren)
		{
			i->printChain(depth + 1);
		}
	}

	bool isDuplicate(const JointRef &ref)
	{
		bool ret = false;

		for (auto &i : mChildren)
		{
			// If this joint is already represented in this hierarchy, then return true.
			// Same parent, same child, and same joint name
			if (ref.mBody0 == mRigidBody && ref.mBody1 == i->mRigidBody && ref.mName == i->mJointName )
			{
				ret = true;
				break;
			}
		}
		if (!ret)
		{
			for (auto &i : mChildren)
			{
				if (i->isDuplicate(ref))
				{
					ret = true;
					break;
				}
			}
		}


		return ret;
	}

	void getJointRef(JointRefVector &joints)
	{
		for (auto &i : mChildren)
		{
			JointRef j;
			j.mBody0 = mRigidBody;
			j.mBody1 = i->mRigidBody;
			j.mName = i->mJointName;
			j.mUsed = false; // not used yet..
			joints.push_back(j);
		}
		for (auto &i : mChildren)
		{
			i->getJointRef(joints);
		}
	}

	// Return the number of children links
	virtual uint32_t getChildCount(void) const override final
	{
		return uint32_t(mChildren.size());
	}

	// Returns the name of the joint and bodies for this child
	virtual const char *getJoint(uint32_t index, const char *&body0, const char *&body1, bool &isLoopJoint) const override final
	{
		const char *ret = nullptr;

		if (index < mChildren.size() )
		{
			const Link *l = mChildren[index];	// Get this child
			ret = l->mJointName.c_str();		// Get the name of this joint
			body0 = mRigidBody.c_str();			// Get parent rigid body name
			body1 = l->mRigidBody.c_str();		// get child rigid body name
			isLoopJoint = l->mIsLoopJoint;		// Set flag to indicate if this is a loop joint
		}

		return ret;
	}

	// Returns this child hierarchy link
	virtual const HierarchyLink *getChild(uint32_t index) const override final
	{
		const HierarchyLink *ret = nullptr;

		if (index < mChildren.size())
		{
			ret = static_cast<const HierarchyLink *>(mChildren[index]);
		}

		return ret;
	}

	void getLinks(LinkVector &links)
	{
		links.push_back(this);
		for (auto &i : mChildren)
		{
			i->getLinks(links);
		}
	}

	virtual const char *getRigidBody(void) const override final
	{
		return mRigidBody.c_str();
	}

	bool			mIsLoopJoint{ false };
	std::string		mJointName;		// Empty string if the root node
	std::string		mRigidBody;
	LinkVector		mChildren;
};

class Hierarchy
{
public:
	Hierarchy(const JointRef &joint,size_t index) : mIndex(index)
	{
		mRoot					= new Link;
		mRoot->mRigidBody		= joint.mBody0;

		Link *firstChild		= new Link;
		firstChild->mJointName	= joint.mName;
		firstChild->mRigidBody	= joint.mBody1;

		mRoot->mChildren.push_back(firstChild);

#if LOG_CHAIN
		debugPrint();
#endif
	}

	virtual ~Hierarchy(void)
	{
		delete mRoot;
	}

	bool merge(Hierarchy *other)
	{
		bool ret = false;

		JointRefVector joints;
		other->mRoot->getJointRef(joints); 
		// get all of the joints in the 'other' hierarchy
		// Keep iterating on this set of joints while we are successfully adding them
		// If any joint intersects, then *all* joints should be able to be successfully added.
		// If not, something is wrong with the design here.
		bool consumed = true;
		// While are consuming input joints from this articulation
		while (consumed)
		{
			consumed = false;
			// Iterate through all joints not already merged 
			for (auto &i : joints)
			{
				if (!i.mUsed)
				{
					if (mRoot->isDuplicate(i))
					{
						i.mUsed = true;
					}
					else if (added(i))
					{
						ret = true;
						i.mUsed = true;
						consumed = true;
					}
				}
			}
		}
		// validation.. if we merged, then 100% of all of the joints should have been consumed..
#ifdef _DEBUG
		if (ret)
		{
			for (auto &i : joints)
			{
				assert(i.mUsed);
			}
		}
#endif
		return ret;
	}

	// Must find loop joints in the same order they were originally defined!
	void findLoopJoints(const JointRefVector &joints)
	{
		// Get the list of links as a flat array...
		LinkVector links;
		mRoot->getLinks(links);
		// We are going to now sort them in order..
		LinkVector sortLinks;
		for (auto &i : joints)
		{
			// for each original source joint...
			for (auto &j : links)
			{
				if (j->mJointName == i.mName)
				{
					sortLinks.push_back(j);
					break;
				}
			}
		}
		StringVector rigidBodies;
		rigidBodies.push_back(mRoot->mRigidBody); // add the root node rigid body
		for (auto &i : sortLinks)
		{
			bool isLoopJoint = false;
			for (auto &j : rigidBodies)
			{
				if (j == i->mRigidBody)
				{
					isLoopJoint = true;
					break;
				}
			}
			if (isLoopJoint)
			{
				i->mIsLoopJoint = true;
			}
			else
			{
				rigidBodies.push_back(i->mRigidBody);	// add this rigid body to the list of rigid bodies..
			}
		}
	}

	void debugPrint(void)
	{
		printf("==========================================================\r\n");
		printf("Hierarchy[%d] with root node of: %s\r\n", 
			uint32_t(mIndex),
			mRoot->mRigidBody.c_str());
		printf("==========================================================\r\n");
		mRoot->printChain(0);
		printf("==========================================================\r\n");
		printf("\r\n");
	}

	bool added(const JointRef &jref)
	{
		bool ret = false;

		Link *l = mRoot->add(jref);
		ret = l ? true : false;
#if LOG_CHAIN
		if (ret)
		{
			printf("==========================================================\r\n");
			printf("Adding: %s-%s\r\n", jref.mBody0.c_str(), jref.mBody1.c_str());
			printf("==========================================================\r\n");
			debugPrint();
			printf("==========================================================\r\n");
			printf("\r\n");
		}
#endif
		return ret;
	}

	const HierarchyLink * getHierarchyRoot(void) const
	{
		return static_cast<const HierarchyLink *>(mRoot);
	}

	size_t		mIndex{ 0 };
	Link		*mRoot{ nullptr };
};

typedef std::vector< Hierarchy *> HierarchyVector;

class HierarchyBuilderImpl : public HierarchyBuilder
{
public:
	HierarchyBuilderImpl(void)
	{

	}

	virtual ~HierarchyBuilderImpl(void)
	{
		reset();
	}

	virtual void reset(void) override final	// reset back to initial state
	{
		for (auto &i : mHierarchies)
		{
			delete i;
		}
		mHierarchies.clear();
		mRigidBodies.clear();
		mJoints.clear();
		mDisconnectedRigidBodies.clear();
	}

	virtual bool addRigidBody(const char *_id) override final	// add a reference to a rigid body by name
	{
		bool ret = false;

		std::string id(_id);
		if ( !findRigidBodRef(id) )
		{
			RigidBodyRef r;
			r.mName = id;
			mRigidBodies.push_back(r);
			ret = true;
		}

		return ret;
	}

	virtual bool addJoint(const char *_jointId,const char *body0, const char *body1) override final // add a reference to a joint that connects two rigid bodies
	{
		bool ret = false;

		std::string jointId(_jointId);

		if ( !findJointRef(jointId ))
		{
			// We cannot add a joint unless it refers to known existing rigid bodies
			if (findRigidBodRef(body0) && findRigidBodRef(body1))
			{
				JointRef j;
				j.mName = jointId;
				j.mBody0 = std::string(body0);
				j.mBody1 = std::string(body1);
				mJoints.push_back(j);
				ret = true;
			}
		}

		return ret;
	}

	JointRef *findFirstUnusedJoint(void)
	{
		JointRef *ret = nullptr;

		for (auto &i : mJoints)
		{
			if (!i.mUsed)
			{
				ret = &i;
				break;
			}
		}
		return ret;
	}

	// Build the hierarchy and return the number of unique hierarchies found
	virtual uint32_t build(void) override final
	{
		// Step number one, identify all rigid bodies which are not referenced by any joint
		// and add them to the disconnected rigid bodies list
		checkForDisconnectedRigidBodies();
		// Now we try to insert every single joint into an existing hierarchy or, if none fit, start a 
		// new one
		JointRef *jref = findFirstUnusedJoint();
		while (jref)
		{
			bool consumed = false;
			for (auto &i : mHierarchies)
			{
				if (i->added(*jref))
				{
					consumed = true;
					break;
				}
			}
			jref->mUsed = true;
			if (!consumed)
			{
				Hierarchy *h = new Hierarchy(*jref, mHierarchies.size());
				mHierarchies.push_back(h);
			}
			jref = findFirstUnusedJoint();
		}
		// Once we have added all of the joints, some of the hierarchies may be fragments
		// This can occur if the joints were added in essentially a randomized order.
		// This pass we see if any hierarchy fragments can be merged into one single chain
		// We continue merging until no more merges can happen
		// We only perform this operation if there is more than one hierarchy found
		if (mHierarchies.size() > 1) // if we ended up with more than one hierarchy, see if they can be merged into a continguous single hierarachy
		{
			uint32_t mergeCount = 0;
			bool mergePass = true;
			// While we are still merging
			while (mergePass)
			{
				mergePass = false;
				for (size_t i = 0; i < mHierarchies.size() && !mergePass; i++)
				{
					Hierarchy *source = mHierarchies[i];
					if (source)
					{
						for (size_t j = i + 1; j < mHierarchies.size(); j++)
						{
							Hierarchy *dest = mHierarchies[j];
							if (dest && source->merge(dest))
							{
								mergePass = true;
								delete dest;
								mHierarchies[j] = nullptr;
								mergeCount++;
							}
						}
					}
				}
			}
			// If any hierarchies were merged, we rebuild the hierarchy list to remove the
			// ones which got merged
			if (mergeCount)
			{
				// After the merge operation, we rebuild the articulations array with just the merged results
				HierarchyVector temp = mHierarchies;
				mHierarchies.clear();
				for (auto i : temp)
				{
					if (i)
					{
						mHierarchies.push_back(i);
					}
				}
			}
		}
		// Search for and flag any loop joints in each hierarchy
		for (auto &i : mHierarchies)
		{
			i->findLoopJoints(mJoints);
		}

		return uint32_t(mHierarchies.size());
	}

	virtual void release(void) override final
	{
		delete this;
	}

	void checkForDisconnectedRigidBodies(void)
	{
		mDisconnectedRigidBodies.clear();
		for (auto &i : mRigidBodies)
		{
			i.mUsed = false;
		}
		for (auto &i : mJoints)
		{
			// for each joint, find any rigid bodies it refers to and mark them as 'used'
			for (auto &j : mRigidBodies)
			{
				if (!j.mUsed)
				{
					// If this rigid body matches the name of either rigid bodies referenced by this
					// joint, then flag the rigid body as 'used'
					if (j.mName == i.mBody0 ||
						j.mName == i.mBody1)
					{
						j.mUsed = true; // mark it as being referenced by a joint
					}
				}
			}
		}
		for (auto &i : mRigidBodies)
		{
			if (!i.mUsed)
			{
				mDisconnectedRigidBodies.push_back(i.mName);
			}
		}

	}

	// Returns the number of rigid bodies which were not connected by any joints
	virtual uint32_t getDisconnectedRigidBodyCount(void) override final
	{
		return uint32_t(mDisconnectedRigidBodies.size());
	}

	// Returns the name of this disconnected rigid body; null of this index is out of range
	virtual const char * getDisconnectedRigidBody(uint32_t index) override final
	{
		const char *ret = nullptr;

		if (index < mDisconnectedRigidBodies.size())
		{
			ret = mDisconnectedRigidBodies[index].c_str();
		}

		return ret;
	}

	void showHierarchy(const HIERARCHY_BUILDER::HierarchyLink *link, uint32_t depth)
	{
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
			printf("%s : %s->%s : loop(%s)\r\n", j, body0, body1, isLoopJoint ? "true" : "false");
		}
		for (uint32_t i = 0; i < count; i++)
		{
			const HIERARCHY_BUILDER::HierarchyLink *child = link->getChild(i);
			showHierarchy(child, depth + 1);
		}
	}

	// Debug printf the results
	// Also works as an example of how to query the results
	virtual void debugPrint(void) final override
	{
		uint32_t count = getDisconnectedRigidBodyCount();
		printf("DisconnectedRigidBodyCount: %d\r\n", count);
		for (uint32_t i = 0; i < count; i++)
		{
			printf("    RigidBody[%d]=%s\r\n", i, getDisconnectedRigidBody(i));
		}
		uint32_t hcount = getHierarchyCount();
		printf("Found %d hierarchies\r\n", hcount);
		for (uint32_t i = 0; i < hcount; i++)
		{
			printf("========================================================\r\n");
			printf("Hierarchy[%d]\r\n", i);
			printf("========================================================\r\n");
			const HIERARCHY_BUILDER::HierarchyLink *link = getHierarchyRoot(i);
			showHierarchy(link, 0);
			printf("========================================================\r\n");
			printf("\r\n");
		}
	}

	JointRef *findJointRef(const std::string &id)
	{
		JointRef *ret = nullptr;

		for (auto &i : mJoints)
		{
			if (i.mName == id)
			{
				ret = &i;
				break;
			}
		}

		return ret;
	}

	RigidBodyRef * findRigidBodRef(const std::string &id)
	{
		RigidBodyRef *ret = nullptr;

		for (auto &i : mRigidBodies)
		{
			if (i.mName == id)
			{
				ret = &i;
				break;
			}
		}

		return ret;
	}

	// returns the number of hierarchies found
	virtual uint32_t getHierarchyCount(void) const override final
	{
		return uint32_t(mHierarchies.size());
	}

	// Return the root link of this hierarchy
	virtual const HierarchyLink * getHierarchyRoot(uint32_t index) const override final
	{
		const HierarchyLink *ret = nullptr;

		if (index < mHierarchies.size())
		{
			ret = static_cast<const HierarchyLink *>(mHierarchies[index]->getHierarchyRoot());
		}

		return ret;
	}

	// Return the number of rigid bodies in the system
	virtual uint32_t getRigidBodyCount(void) override final
	{
		return uint32_t(mRigidBodies.size());
	}

	// Return the name of a rigid body input
	virtual const char *getRigidBody(uint32_t index) override final
	{
		const char *ret = nullptr;
		if (index < mRigidBodies.size())
		{
			ret = mRigidBodies[index].mName.c_str();
		}
		return ret;
	}

	// Return the number of joints in the system
	virtual uint32_t getJointCount(void) override final
	{
		return uint32_t(mJoints.size());
	}

	// Return the name of a joint input and the names of the bodies it connects
	virtual const char *getJoint(uint32_t index, const char *&body0, const char *&body1) override final
	{
		const char *ret = nullptr;

		body0 = nullptr;
		body1 = nullptr;
		if (index < mJoints.size())
		{
			const JointRef &j = mJoints[index];
			ret = j.mName.c_str();
			body0 = j.mBody0.c_str();
			body1 = j.mBody1.c_str();
		}

		return ret;
	}


private:
	RigidBodyRefVector	mRigidBodies;		// Raw collection of source rigid bodies that may, or may not, be connected by joints
	JointRefVector		mJoints;			// Raw collection of source joints
	HierarchyVector		mHierarchies;		// number of unique hierarchies found
	StringVector		mDisconnectedRigidBodies;
};

HierarchyBuilder *HierarchyBuilder::create(void)
{
	auto ret = new HierarchyBuilderImpl;
	return static_cast<HierarchyBuilder *>(ret);
}


} // end of namespace


