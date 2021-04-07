#ifndef _BEHAVIAC_GROUPASBASICLOGIC_H_
#define _BEHAVIAC_GROUPASBASICLOGIC_H_

#include "behaviac_headers.h"
#include "BlackBoard.h"
#include "TaskRealize.h"

class BlackBoard;
class TaskRealize;
extern BlackBoard* g_BlackBoardAgent;
extern TaskRealize* g_TaskRealizeAgent;


class GroupAsBasicLogic : public behaviac::Agent
{
public:
	GroupAsBasicLogic();
	virtual ~GroupAsBasicLogic();

	BEHAVIAC_DECLARE_AGENTTYPE(GroupAsBasicLogic, behaviac::Agent)

	public: TaskIndividual CurrentTask;

	public: behaviac::vector<int> GroupMember;

	public: behaviac::vector<ForeFuncState> GroupState;

	public:behaviac::vector<behaviac::vector<int>>MembersFromOtherMembers;

	public: bool wait_for_CB;

	public: void ActionCancel();

	public: bool GroupIdle();

	public: bool IsForegrdFunc(const TaskIndividual& Task);

	public: bool MemberConsistent();

	public: void RealTimeProcessing();

	public: void SendGoal();

	public: void SetMemberAndGoal();

	public: bool TaskListEmpty();

	public: TaskIndividual TaskListPop();

};


#endif
