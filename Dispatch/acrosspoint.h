#ifndef ACROSSPOINT_H
#define ACROSSPOINT_H

#include "AStar.hpp"
#include "combpath.h"

using namespace HRGU;

class Path;
class Map;
class AcrossPoint : public Vec2i
{
public:
    AcrossPoint();
    ~AcrossPoint();

    int  m_dID;
    int  GetJoinNum();

    bool                     m_bAdjustModifyPosture;
    bool                     m_bOccupyLock;
    std::vector<bool>        m_bDirection;
    std::vector<Path*>       m_pPaths;

    //topology map     start
    bool         OccupyLock();
    bool         ReleaseLock();
    bool         CheckLock();

    std::vector<ApplyInfor>  m_tempApplyList;

    int          m_dStepID;

    Map*         m_pMap;

    void         DealMidApply(int dMaxLength);
    void         dealMidApply_SortApplyByPriority(std::vector<ApplyInfor>  &UndealApplyList, int dMaxLength);
    void         dealMidApply_ChooseaFeasibleApply(std::vector<ApplyInfor>  &UndealApplyList);
    bool         dealMidApply_CheckManageableConflictApply(std::vector<ApplyInfor>  &UndealApplyList, int dMaxLength,  ApplyInfor &DodgeAgentApply);
    void         dealMidApply_ApplyPass(ApplyInfor  &pProcessingApply);
    void         dealMidApply_ConflictCase1(std::vector<Path*>  PossiblePathList,  Agent* pAGV1, Agent* pAGV2,
                                            Path* pPath1, Path* pPath2, ApplyInfor AGV1ProcApply, ApplyInfor AGV2NewApply);
    void         dealMidApply_ConflictCase2( Agent* pAGV1,  Agent* pAGV2, Path*  pPath1, Path*  pPath2, Path*  pPath3);
    void         GetAllNoUsedPath(std::vector<Path*> &pathlist);
    void         StaticPathNum();
    int          GetAGVNumberOfThisZone();
    int          GetPathNumberOfThisZone();
    int          m_dPathNum;
    //topology map     end

private:
    //ApplyInfor*  m_pProcessingApply;
    ApplyInfor  m_ApplyProcessing;
};



#endif // ACROSSPOINT_H
