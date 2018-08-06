#include "acrosspoint.h"
#include "path.h"
#include "agent.h"
#include "task.h"
#include "map.h"
#include "manager.h"

typedef pair<Path*, float> PAIR;

struct CmpByValue
{
  bool operator()(const PAIR& lhs, const PAIR& rhs)
  {
    return lhs.second > rhs.second;
  }
};


AcrossPoint::AcrossPoint()
{
    for(int i =0; i< 8; i++)
    {
        m_bDirection.push_back(false);
        m_pPaths.push_back(NULL);
    }
    //sem_init(&m_uOccupyLock,0,1);
    m_bOccupyLock = false;
    m_bAdjustModifyPosture = false;

    m_tempApplyList.clear();
    //m_pProcessingApply = NULL;
    m_ApplyProcessing.pAgent    = NULL;
    m_ApplyProcessing.pCombPath = NULL;

    m_pMap             = NULL;

    m_dStepID  = 0;
    m_dPathNum = 0;
}

AcrossPoint::~AcrossPoint()
{
    for(int i =0; i< 8; i++)
    {
        if(m_pPaths[i] != NULL)
        {
            free(m_pPaths[i]);
            m_pPaths[i] =NULL;
        }
    }
    //sem_destroy(&m_uOccupyLock);
}

int  AcrossPoint::GetJoinNum()
{
    int    dJoinNum = m_bDirection[0] + m_bDirection[1] + m_bDirection[2] + m_bDirection[3] +
                      m_bDirection[4] + m_bDirection[5] + m_bDirection[6] + m_bDirection[7];
    return dJoinNum;
}

bool  AcrossPoint::OccupyLock()
{
    bool   flag = false;
    if( m_bOccupyLock == false )
    {
        m_bOccupyLock = true;
        flag    = true;
    }

    return  flag;
}

bool  AcrossPoint::ReleaseLock()
{
    bool   flag = false;
    if(m_bOccupyLock == true)
    {
        m_bOccupyLock = false;
        flag    = true;
    }

    return flag;
}

bool  AcrossPoint::CheckLock()
{
    return m_bOccupyLock;
}



void   AcrossPoint::dealMidApply_SortApplyByPriority(std::vector<ApplyInfor>  &UndealApplyList, int dMaxLength)
{
    //sort apply by its' taskgroup's Priority
    if( UndealApplyList.size() > 1 )
    {

        for(int i = 0 ; i< (UndealApplyList.size() -1); i++)
        {
            int   dMaxid       = -1;
            float fMaxPriority = -1;
            int   j = i;
            for( ; j < UndealApplyList.size(); j++)
            {
                Agent*         pAgent      = UndealApplyList[j].pAgent;
                RealTaskGroup* pTaskGroup  = pAgent->m_pTaskGroup;
                Task*          pTaskHead   = pTaskGroup->m_pTaskHead;
                Path*          pOccupyPath = pTaskHead->GetExecuteCombPath()->m_pPath;

                float  fPriority = pTaskGroup->m_dPriority + pTaskHead->PriorityTactics(dMaxLength) + (pOccupyPath->m_bCouldUsedForDodge ? 0 : 10);
                if( fPriority > fMaxPriority )
                {
                    dMaxid = j;
                    fMaxPriority = fPriority;
                }
            }

            if(dMaxid != i)
                std::swap(UndealApplyList[i], UndealApplyList[dMaxid]);
        }
    }
}

void   AcrossPoint::dealMidApply_ChooseaFeasibleApply( std::vector<ApplyInfor>  &UndealApplyList )
{
    if( UndealApplyList.size() > 0 ) //if have some application not be processed
    {
        //choose the feasible apply as ProcessingApply application
        for(int i =0; i < UndealApplyList.size(); i++)
        {
            CombPath*   pApplyTCombPath = UndealApplyList[i].pCombPath;
            Agent*      pAgent          = UndealApplyList[i].pAgent;
            Path*       pApplyPath      = pApplyTCombPath->m_pPath;

            bool   bCondition_1 = pApplyPath->m_bNextOccupyLock.CheckLock();
            bool   bCondition_2 = pApplyPath->m_bOccupyLock.CheckLock();

            Agent* pOccupyAgent     = NULL;
            pApplyPath->m_bOccupyLock.GetOccupySub(pOccupyAgent);
            Agent* pNextOccupyAgent = NULL;
            pApplyPath->m_bNextOccupyLock.GetOccupySub(pNextOccupyAgent);

            if( !bCondition_1  && !bCondition_2 )
            {

                /*  if apply path have double across, then we must be sure that the other across' path number must bigger than AGV's  */
                if( !(pApplyPath->m_bMonopoly) )
                {
                    int  OtherAcrossID = ( pApplyPath->m_dAcrossID[0] == m_dID ) ? pApplyPath->m_dAcrossID[1] : pApplyPath->m_dAcrossID[0];
                    AcrossPoint*  pAcrossHead = NULL;
                    for(int i =0 ; i< m_pMap->m_AcrossPointList.size(); i++)
                    {
                        if(m_pMap->m_AcrossPointList[i]->m_dID == OtherAcrossID)
                        {
                            pAcrossHead = m_pMap->m_AcrossPointList[i];
                            break;
                        }
                    }

                    int  dAGVNum  = pAcrossHead->GetAGVNumberOfThisZone();
                    int  dPathNum = pAcrossHead->GetPathNumberOfThisZone();
                    if(dAGVNum < dPathNum)
                    {
                        OccupyLock();
                        pApplyTCombPath->ChangeStatue( COMBPATH_MOVE_ACROSS );
                        pApplyPath->m_dDirection = pApplyTCombPath->m_dDirection;
                        pApplyPath->m_bOccupyLock.OccupyLock(pAgent);
                        //m_pProcessingApply = &(UndealApplyList[i]);
                        m_ApplyProcessing = UndealApplyList[i];
                    }
                }
                else
                {
                    OccupyLock();
                    pApplyTCombPath->ChangeStatue( COMBPATH_MOVE_ACROSS );
                    pApplyPath->m_dDirection = pApplyTCombPath->m_dDirection;
                    pApplyPath->m_bOccupyLock.OccupyLock(pAgent);
                    //m_pProcessingApply = &(UndealApplyList[i]);
                    m_ApplyProcessing = UndealApplyList[i];
                }

            }
            else if( !bCondition_1 && bCondition_2 && (pOccupyAgent == pAgent) ) //the dodge agent, has a empty dodge path, had occupy this path alreday
            {
                OccupyLock();
                pApplyTCombPath->ChangeStatue( COMBPATH_MOVE_ACROSS );
                pApplyPath->m_dDirection = pApplyTCombPath->m_dDirection;
                //m_pProcessingApply = &(UndealApplyList[i]);
                m_ApplyProcessing = UndealApplyList[i];
            }
            else if(  bCondition_1                                        &&
                     !bCondition_2                                        &&
                     (pAgent == pNextOccupyAgent)                         &&
                     ( TASK_STATUE_MOVEING == pAgent->m_pTaskGroup->GetCurrentTask()->GetStatue() ) &&
                     (pApplyPath->m_AGVOccupyList.size() == 0) )   //occupy ApplyPath in advance,but not dodge mode
            {
                OccupyLock();
                pApplyTCombPath->ChangeStatue( COMBPATH_MOVE_ACROSS );
                pApplyPath->m_dDirection = pApplyTCombPath->m_dDirection;
                //m_pProcessingApply = &(UndealApplyList[i]);
                m_ApplyProcessing = UndealApplyList[i];
            }
            else if(  bCondition_1                                        &&
                      bCondition_2                                        &&
                     (pAgent == pNextOccupyAgent)                         &&
                     (pAgent->m_pTaskGroup->m_TempTaskList.size() > 0)    &&  //dodge agent,this path have a agent occupy alreday
                     (pApplyPath->m_AGVOccupyList.size() == 1) )
            {
                OccupyLock();
                pApplyTCombPath->ChangeStatue( COMBPATH_MOVE_ACROSS );
                pApplyPath->m_dDirection = pApplyTCombPath->m_dDirection;
                //m_pProcessingApply = &(UndealApplyList[i]);
                m_ApplyProcessing = UndealApplyList[i];
            }

            if( ( m_ApplyProcessing.pAgent != NULL ) && (m_ApplyProcessing.pCombPath != NULL) )
                break;
        }

    }
}

bool   AcrossPoint::dealMidApply_CheckManageableConflictApply(std::vector<ApplyInfor>  &UndealApplyList, int dMaxLength, ApplyInfor &DodgeAgentApply )
{
    Agent*      pApplyAgent     = UndealApplyList[0].pAgent;
    CombPath*   pApplyTCombPath = UndealApplyList[0].pCombPath;
    Path*       pTargetPath     = pApplyTCombPath->m_pPath;
    Path*       pApplyAgentCurOcpPath    = pApplyAgent->m_pTaskGroup->m_pTaskHead->GetExecuteCombPath()->m_pPath;

    //ApplyAgentCurOcpPath must not been occupy by other agent in advance
/*
    bool bcondition_0 = false;
    if(!( pApplyAgentCurOcpPath->m_bNextOccupyLock.CheckLock() ))
        bcondition_0 = true;
    if( !bcondition_0 )
        return false;
*/

    //TargetPath must not been occupy by other agent in advance
    bool bcondition_1 = false;
    if(!( pTargetPath->m_bNextOccupyLock.CheckLock() ))
        bcondition_1 = true;
    if( !bcondition_1 )
        return false;

    float fPrority_ApplyPathTask      = pTargetPath->GetMaxPriority(dMaxLength);
    float fPrority_ApplyAgentCurTask  = pApplyAgentCurOcpPath->GetMaxPriority(dMaxLength);
    //Occupy Path's Prority must big than Target path's Prority
    bool bcondition_2 = (fPrority_ApplyAgentCurTask >= fPrority_ApplyPathTask) ? true : false;
    if( !bcondition_2 )
        return false;

    //Target path's AGV must move toward to this across
    bool     bcondition_3 = false;
    int dCurDirection = pTargetPath->m_dDirection;
    if(pApplyTCombPath->m_dDirection != dCurDirection)
         bcondition_3 = true;
    if( !bcondition_3 )
        return false;

    //Target path's AGV must finish current combpath and send apply alreday
    bool     bcondition_4 = false;
    if(pTargetPath->m_AGVOccupyList.size() > 0)
    {
        Agent*   pDodgeAgent  = pTargetPath->m_AGVOccupyList[0];

        for( int i =0; i < UndealApplyList.size(); i++ )
        {
            if( ( UndealApplyList[i].pAgent == pDodgeAgent ) && (COMBPATH_WAIT_DISPATCH == UndealApplyList[i].pCombPath->GetStatue()) )
            {
                bcondition_4 = true;
                DodgeAgentApply = UndealApplyList[i];
                break;
            }
        }
    }
    if( !bcondition_4 )
        return false;

    return true;
}


void   AcrossPoint::dealMidApply_ConflictCase1(std::vector<Path*>  PossiblePathList,  Agent* pAGV1, Agent* pAGV2,
                                               Path* pPath1, Path* pPath2, ApplyInfor AGV1ProcApply, ApplyInfor AGV2NewApply)
{
    map<Path*,float>    scoreAndPath;
    for( int i =0 ; i < PossiblePathList.size(); i++ )
    {
        float      dScore = 0;
        Path*    pPathHead = PossiblePathList[i];
        bool     bCurrLock = pPathHead->m_bOccupyLock.CheckLock();
        if(!bCurrLock)
            dScore += 10;
        else
        {
            Agent*   pAGV = NULL;
            pPathHead->m_bOccupyLock.GetOccupySub(pAGV);
            AgentState eState= pAGV->GetState();

            switch(eState)
            {
            case AGENT_STATE_WAIT:
            {
                std::vector<Vec2i>    DodgePos;
                std::vector<Agent*>   pAGVList;
                std::vector<int>     dDodgeDirection;
                int dAGV2Direction = ( m_dID == pPathHead->m_dAcrossID[0] ) ? 1 : 0;
                int dAGV3Direction = (dAGV2Direction + 1)%2;
                dDodgeDirection.push_back(dAGV3Direction);
                dDodgeDirection.push_back(dAGV2Direction);

                Vec2i   AGV3Pos = pAGV->GetCurrentGridPosition();
                DodgePos.push_back( AGV3Pos );

                pAGVList.push_back(pAGV);
                pAGVList.push_back(pAGV2);

                /*   int   AGV2Posture  = pAGV2->CalcAgentNextPosture( (Vec2i)*(this) );   */
                int   AGV2Posture  = 180;
                Vec2i AGV3CurPos   = pAGV->GetCurrentGridPosition();
                Vec2i AGV3NexPos;
                int   AGV3CurPosID = pPathHead->GetSpeGridPosIndexWithDirection(AGV3CurPos, dAGV3Direction);
                if( AGV3CurPosID < (pPathHead->GetPathSize()-1) )
                    AGV3NexPos = pPathHead->GetGridPosFromIndexDirection((AGV3CurPosID +1), dAGV3Direction);
                else
                    AGV3NexPos = (Vec2i)*(this);
                int   AGV3Posture  = pAGV2->CalcAgentNextPosture(AGV3NexPos);

                std::vector<int> PostureList;
                PostureList.push_back(AGV3Posture);
                PostureList.push_back(AGV2Posture);

                bool  dFlag = pPathHead->GetDodgePos( dDodgeDirection, DodgePos, pAGVList, PostureList);
                if(dFlag)
                    dScore += 1;
            }
                break;
            case AGENT_STATE_DEALINGTASK:
            {
                Task*      pCurtask = pAGV->m_pTaskGroup->GetCurrentTask();
                TaskStatue  eStatue = pCurtask->GetStatue();
                switch(eStatue)
                {
                case TASK_STATUE_MOVEING:
                    dScore += 5;
                    break;
                case TASK_STATUE_PROCESS:
                    dScore += 3;
                    break;
                default:
                    break;
                }
            }
                break;
            default:
                break;
            }
        }
        scoreAndPath.insert(make_pair(pPathHead, dScore));
    }
    std::vector<PAIR> head_score_vec(scoreAndPath.begin(), scoreAndPath.end());
    std::sort(head_score_vec.begin(), head_score_vec.end(), CmpByValue());

    vector<PAIR>     PossiblePathList_Cond2_MOVE;  //this path3's occupy agent must not in processing mode
    for(int i =0; i < head_score_vec.size(); i++)
    {
        if(head_score_vec[i].second > 0 )
            PossiblePathList_Cond2_MOVE.push_back(head_score_vec[i]);
    }
    if( PossiblePathList_Cond2_MOVE.size() == 0 )
        return;

    Path*     pPath3 = PossiblePathList_Cond2_MOVE[0].first;
    float     fScore = PossiblePathList_Cond2_MOVE[0].second;
    Agent*    pAGV3  = pPath3->m_AGVOccupyList[0];
    //delete AGV3's new Apply
    bool      bAGV3NeedModify = true;
    CombPath  *pAGV3CurCombPath = pAGV3->m_pTaskGroup->GetCurrentTask()->GetExecuteCombPath();
    if(fScore > 1) //AGV3 task don't finish
    {
        CombPathStatue  eStatue = pAGV3CurCombPath->GetStatue();

        switch(eStatue)
        {
        case COMBPATH_MOVE_ACROSS:
        case COMBPATH_MOVE_PATH:
            {
                if( pAGV3CurCombPath->m_pPath->m_dAcrossID[pAGV3CurCombPath->m_dDirection] != m_dID ) //away this across
                    bAGV3NeedModify = false;
            }
            break;
        case COMBPATH_MOVE_PATH_DECELERATE://delete AGV3's new Apply
        case COMBPATH_WAIT_NEXT_APPROVE:
            {
                if( pAGV3CurCombPath->m_pPath->m_dAcrossID[pAGV3CurCombPath->m_dDirection] != m_dID )
                    bAGV3NeedModify = false;
                else
                {
                    m_pMap->DeleteNextApply(pAGV3, pAGV3CurCombPath);
                    pAGV3CurCombPath->SetStatue( COMBPATH_MOVE_PATH );
                }
            }
            break;
        }
    }
    else
        bAGV3NeedModify = false;

    //delete AGV2's new Apply
    CombPath* pAGV2CurCombPath = pAGV2->m_pTaskGroup->GetCurrentTask()->GetExecuteCombPath();
    m_pMap->DeleteNextApply(pAGV2, pAGV2CurCombPath);

    pPath3->m_bNextOccupyLock.OccupyLock( pAGV2 );
    //change Target Path's Occupy Agent's task(modify this agent's trail)

    std::vector<Vec2i>    DodgePos;
    std::vector<Agent*>   pAGVList;
    if(bAGV3NeedModify)
    {
        std::vector<int>     dDodgeDirection;
        int dAGV2Direction = ( m_dID == pPath3->m_dAcrossID[0] ) ? 1 : 0;
        int dAGV3Direction = pAGV3->m_pTaskGroup->GetCurrentTask()->GetExecuteCombPath()->m_dDirection;
        dDodgeDirection.push_back(dAGV3Direction);
        dDodgeDirection.push_back(dAGV2Direction);

        Vec2i   AGV3Pos = pAGV3->GetCurrentGridPosition();
        DodgePos.push_back( AGV3Pos );

        pAGVList.push_back(pAGV3);
        pAGVList.push_back(pAGV2);

        /* int   AGV2Posture  = pAGV2->CalcAgentNextPosture( (Vec2i)*(this) ); */
        int   AGV2Posture  = 180;
        Vec2i AGV3CurPos   = pAGV3->GetCurrentGridPosition();
        Vec2i AGV3NexPos;
        int   AGV3CurPosID = pPath3->GetSpeGridPosIndexWithDirection(AGV3CurPos, dAGV3Direction);
        if( AGV3CurPosID < (pPath3->GetPathSize()-1) )
            AGV3NexPos = pPath3->GetGridPosFromIndexDirection((AGV3CurPosID +1), dAGV3Direction);
        else
        {
            //away this across
            if( pAGV3CurCombPath->m_pPath->m_dAcrossID[pAGV3CurCombPath->m_dDirection] != m_dID )
            {
                int  dAcrossID = pAGV3CurCombPath->m_pPath->m_dAcrossID[pAGV3CurCombPath->m_dDirection];
                AGV3NexPos = (Vec2i)*(m_pMap->m_AcrossPointList[dAcrossID]);
            }
            else   //toward this across
                AGV3NexPos = (Vec2i)*(this);
        }
        int   AGV3Posture  = pAGV3->CalcAgentNextPosture( AGV3NexPos );
        std::vector<int> PostureList;
        PostureList.push_back(AGV3Posture);
        PostureList.push_back(AGV2Posture);

        pPath3->GetDodgePos( dDodgeDirection, DodgePos, pAGVList, PostureList);
    }
    else
    {
        std::vector<int>     dDodgeDirection;
        int dAGV2Direction = ( m_dID == pPath3->m_dAcrossID[0] ) ? 1 : 0;
        dDodgeDirection.push_back(dAGV2Direction);
        pAGVList.push_back(pAGV2);
        /*  int   AGV2Posture  = pAGV2->CalcAgentNextPosture( (Vec2i)*(this) ); */
        int   AGV2Posture = 180;
        std::vector<int> PostureList;
        PostureList.push_back(AGV2Posture);
        pPath3->GetDodgePos( dDodgeDirection, DodgePos, pAGVList, PostureList);
    }


    //modify this agv's taskgroup
    string  sDodgeLockAgv = pAGV1->GetName() + " " + pAGV2->GetName();
    if(bAGV3NeedModify)
        pAGV3->m_pTaskGroup->SetTempTask_OnceOccupyDodge(pPath3, DodgePos[0], pAGV3->m_pTaskGroup->GetCurrentTask()->GetExecuteCombPath(), sDodgeLockAgv);
    pAGV2->m_pTaskGroup->SetTempTask_OnceDodge(AGV1ProcApply.pCombPath->m_pAcross, pPath3, DodgePos[1], pAGV2->m_pTaskGroup->GetCurrentTask()->GetExecuteCombPath(), pAGV1->GetName());
    //ApplyAgent occupy ApplyPath in advance
    pPath2->m_bNextOccupyLock.OccupyLock(pAGV1);
}


void   AcrossPoint::dealMidApply_ConflictCase2( Agent* pAGV1,  Agent* pAGV2, Path*  pPath1,  Path*  pPath2,  Path*  pPath3 )
{
    Agent*   pAGV = NULL;
    float    dScore = 0;
    pPath3->m_bOccupyLock.GetOccupySub(pAGV);
    AgentState eState= pAGV->GetState();
    switch(eState)
    {
    case AGENT_STATE_WAIT:
    {
        std::vector<Vec2i>    DodgePos;
        std::vector<Agent*>   pAGVList;
        std::vector<int>     dDodgeDirection;
        int dAGV2Direction = ( m_dID == pPath3->m_dAcrossID[0] ) ? 1 : 0;
        int dAGV3Direction = (dAGV2Direction + 1)%2;
        dDodgeDirection.push_back(dAGV3Direction);
        dDodgeDirection.push_back(dAGV2Direction);

        Vec2i   AGV3Pos = pAGV->GetCurrentGridPosition();
        DodgePos.push_back( AGV3Pos );

        pAGVList.push_back(pAGV);
        pAGVList.push_back(pAGV2);

        /* int   AGV2Posture  = pAGV2->CalcAgentNextPosture( (Vec2i)*(this) ); */
        int   AGV2Posture  = 180;
        Vec2i AGV3CurPos   = pAGV->GetCurrentGridPosition();
        Vec2i AGV3NexPos;
        int   AGV3CurPosID = pPath3->GetSpeGridPosIndexWithDirection(AGV3CurPos, dAGV3Direction);
        if( AGV3CurPosID < (pPath3->GetPathSize()-1) )
            AGV3NexPos = pPath3->GetGridPosFromIndexDirection((AGV3CurPosID +1), dAGV3Direction);
        else
            AGV3NexPos = (Vec2i)*(this);
        int   AGV3Posture  = pAGV2->CalcAgentNextPosture(AGV3NexPos);

        std::vector<int> PostureList;
        PostureList.push_back(AGV3Posture);
        PostureList.push_back(AGV2Posture);

        bool  dFlag = pPath3->GetDodgePos( dDodgeDirection, DodgePos, pAGVList, PostureList);
        if(dFlag)
            dScore += 1;
    }
        break;
    case AGENT_STATE_DEALINGTASK:
    {
        Task*      pCurtask = pAGV->m_pTaskGroup->GetCurrentTask();
        TaskStatue  eStatue = pCurtask->GetStatue();
        switch(eStatue)
        {
        case TASK_STATUE_MOVEING:
            dScore += 5;
            break;
        case TASK_STATUE_PROCESS:
            dScore += 3;
            break;
        default:
            break;
        }
    }
        break;
    default:
        break;
    }


    Agent*  pAGV3  = pPath3->m_AGVOccupyList[0];
    //delete AGV3's new Apply
    bool bAGV3NeedModify = true;
    CombPath  *pAGV3CurCombPath = pAGV3->m_pTaskGroup->GetCurrentTask()->GetExecuteCombPath();
    if(dScore > 1) //AGV3 task don't finish
    {
        CombPathStatue  eStatue = pAGV3CurCombPath->GetStatue();

        switch(eStatue)
        {
        case COMBPATH_MOVE_ACROSS:
        case COMBPATH_MOVE_PATH:
            {
                if( pAGV3CurCombPath->m_pPath->m_dAcrossID[pAGV3CurCombPath->m_dDirection] != m_dID )
                    bAGV3NeedModify = false;
            }
            break;
        case COMBPATH_MOVE_PATH_DECELERATE://delete AGV3's new Apply
        case COMBPATH_WAIT_NEXT_APPROVE:
            {
                if( pAGV3CurCombPath->m_pPath->m_dAcrossID[pAGV3CurCombPath->m_dDirection] != m_dID )
                    bAGV3NeedModify = false;
                else
                {
                    m_pMap->DeleteNextApply(pAGV3, pAGV3CurCombPath);
                    pAGV3CurCombPath->SetStatue( COMBPATH_MOVE_PATH );
                }
            }
            break;
        }
    }
    else
        bAGV3NeedModify = false;

    //delete AGV2's new Apply
    CombPath* pAGV2CurCombPath = pAGV2->m_pTaskGroup->GetCurrentTask()->GetExecuteCombPath();
    m_pMap->DeleteNextApply(pAGV2, pAGV2CurCombPath);

    pPath3->m_bNextOccupyLock.OccupyLock( pAGV2 );
    //change Target Path's Occupy Agent's task(modify this agent's trail)
    std::vector<int>      dDodgeDirection;
    std::vector<Vec2i>    DodgePos;
    std::vector<Agent*>   pAGVList;
    std::vector<int>      PostureList;
    if(bAGV3NeedModify)
    {
        int dAGV2Direction = ( m_dID == pPath3->m_dAcrossID[0] ) ? 1 : 0;
        int dAGV3Direction = pAGV3->m_pTaskGroup->GetCurrentTask()->GetExecuteCombPath()->m_dDirection;
        dDodgeDirection.push_back(dAGV3Direction);
        dDodgeDirection.push_back(dAGV2Direction);

        Vec2i   AGV3Pos = pAGV3->GetCurrentGridPosition();
        DodgePos.push_back( AGV3Pos );

        pAGVList.push_back(pAGV3);
        pAGVList.push_back(pAGV2);

        /* int   AGV2Posture  = pAGV2->CalcAgentNextPosture( (Vec2i)*(this) ); */
        int   AGV2Posture  = 180;
        Vec2i AGV3CurPos   = pAGV3->GetCurrentGridPosition();
        Vec2i AGV3NexPos;
        int   AGV3CurPosID = pPath3->GetSpeGridPosIndexWithDirection(AGV3CurPos, dAGV3Direction);
        if( AGV3CurPosID < (pPath3->GetPathSize()-1) )
            AGV3NexPos = pPath3->GetGridPosFromIndexDirection((AGV3CurPosID +1), dAGV3Direction);
        else
        {
            //away this across
            if( pAGV3CurCombPath->m_pPath->m_dAcrossID[pAGV3CurCombPath->m_dDirection] != m_dID )
            {
                int  dAcrossID = pAGV3CurCombPath->m_pPath->m_dAcrossID[pAGV3CurCombPath->m_dDirection];
                AGV3NexPos = (Vec2i)*(m_pMap->m_AcrossPointList[dAcrossID]);
            }
            else   //toward this across
                AGV3NexPos = (Vec2i)*(pAGV3CurCombPath->m_pAcross);
        }
        int   AGV3Posture  = pAGV3->CalcAgentNextPosture(AGV3NexPos);
        PostureList.push_back(AGV3Posture);
        PostureList.push_back(AGV2Posture);
        pPath3->GetDodgePos( dDodgeDirection, DodgePos, pAGVList, PostureList);
    }
    else
    {

        int dAGV2Direction = ( m_dID == pPath3->m_dAcrossID[0] ) ? 1 : 0;
        dDodgeDirection.push_back(dAGV2Direction);
        pAGVList.push_back(pAGV2);
        /* int   AGV2Posture  = pAGV2->CalcAgentNextPosture( (Vec2i)*(this) ); */
        int   AGV2Posture  = 180;
        PostureList.push_back(AGV2Posture);
        pPath3->GetDodgePos( dDodgeDirection, DodgePos, pAGVList, PostureList);
    }

    std::vector<int>     dDodgeDirection_2;
    std::vector<Vec2i>   DodgePos_2;
    std::vector<Agent*>  pAGVList_2;
    std::vector<int>     PostureList_2;
    int dAGV2Direction = ( m_dID == pPath1->m_dAcrossID[0] ) ? 1 : 0;
    dDodgeDirection_2.push_back(dAGV2Direction);
    pAGVList_2.push_back(pAGV2);
    /* int   AGV2Posture  = ( PostureList[1] == 0 ) ? 180 : 0;   */
    int   AGV2Posture  = 180;
    PostureList_2.push_back(AGV2Posture);
    pPath1->GetDodgePos( dDodgeDirection_2, DodgePos_2, pAGVList_2, PostureList_2);


    //modify this agv's taskgroup
    string  sDodgeLockAgv = pAGV1->GetName() + " " + pAGV2->GetName();
    Agent*  pDodgePathOccupyAgent = pPath3->m_AGVOccupyList[0];
    if(bAGV3NeedModify)
        pDodgePathOccupyAgent->m_pTaskGroup->SetTempTask_OnceOccupyDodge(pPath3, DodgePos[0], pDodgePathOccupyAgent->m_pTaskGroup->GetCurrentTask()->GetExecuteCombPath(), sDodgeLockAgv);
    pAGV2->m_pTaskGroup->SetTempTask_TwiceDodge(this, pPath3, pPath1, DodgePos[1], DodgePos_2[0], pAGV2->m_pTaskGroup->GetCurrentTask()->GetExecuteCombPath(), pAGV1->GetName());
    //ApplyAgent occupy ApplyPath in advance
    pPath2->m_bNextOccupyLock.OccupyLock(pAGV1);
}

void   AcrossPoint::DealMidApply(int dMaxLength)
{
    if( ( m_ApplyProcessing.pAgent == NULL ) && (m_ApplyProcessing.pCombPath == NULL) && !CheckLock() )
    {
        //classify apply
        std::vector<ApplyInfor>  UndealApplyList;
        std::vector<ApplyInfor>  DealedApplyList;
        //std::vector
        for( int i =0; i < m_tempApplyList.size(); i++ )
        {
            CombPathStatue  eStatue = m_tempApplyList[i].pCombPath->GetStatue();
            if( COMBPATH_WAIT_DISPATCH == eStatue )
                UndealApplyList.push_back(m_tempApplyList[i]);
            else
                DealedApplyList.push_back(m_tempApplyList[i]);
        }

        dealMidApply_SortApplyByPriority(UndealApplyList, dMaxLength);

        if( UndealApplyList.size() > 0 ) //if have some application not be processed
        {
            //choose the feasible apply as ProcessingApply application
            dealMidApply_ChooseaFeasibleApply( UndealApplyList );

            //conflict process: last step could not find a feasible apply, then calculate path's max priority as path's priority
            if( ( m_ApplyProcessing.pAgent == NULL ) && (m_ApplyProcessing.pCombPath == NULL)  )
            {
                ApplyInfor DodgeAgentApply;
                bool bflag = dealMidApply_CheckManageableConflictApply(UndealApplyList, dMaxLength, DodgeAgentApply);
                if(!bflag)
                    return;

                /* AGV1:  processing agent                                               */
                /* AGV2:  dodge agent (which occupy the AGV1's target path)              */
                /* Path1:  AGV1's current occupy path                                    */
                /* Path2:  AGV1's target                                                 */

                Agent*      pAGV1     = UndealApplyList[0].pAgent;
                Agent*      pAGV2     = DodgeAgentApply.pAgent;
                CombPath*   pApplyTCombPath = UndealApplyList[0].pCombPath;
                Path*       pPath1    = pAGV1->m_pTaskGroup->m_pTaskHead->GetExecuteCombPath()->m_pPath;
                Path*       pPath2    = pApplyTCombPath->m_pPath;
                //deal this conflict
                std::vector<Path*>  pIdelPathList;
                pApplyTCombPath->m_pAcross->GetAllNoUsedPath(pIdelPathList);
                if(pIdelPathList.size() != 0)
                {
                    //set this idle path Dodge Lock
                    Path*  pDodgePath   = pIdelPathList[0];
                    pDodgePath->m_bOccupyLock.OccupyLock( pAGV2 );
                    //delete Target Path's Occupy Agent's Apply
                    CombPath* pAGV2CurCombPath = pAGV2->m_pTaskGroup->GetCurrentTask()->GetExecuteCombPath();
                    m_pMap->DeleteNextApply(pAGV2, pAGV2CurCombPath);
                    //change Target Path's Occupy Agent's task(modify this agent's trail)
                    std::vector<int>     dDodgeDirection;
                    std::vector<Vec2i>   DodgePos;
                    std::vector<Agent*>  pAGVList;
                    std::vector<int>     PostureList;
                    int dAGV2Direction = ( m_dID == pDodgePath->m_dAcrossID[0] ) ? 1 : 0;
                    dDodgeDirection.push_back(dAGV2Direction);
                    pAGVList.push_back(pAGV2);
/*
                    int   AGV2Posture  = pAGV2->CalcAgentNextPosture( (Vec2i)*(this) );
*/
                    int   AGV2Posture  = 180;
                    PostureList.push_back(AGV2Posture);
                    pDodgePath->GetDodgePos( dDodgeDirection, DodgePos, pAGVList, PostureList);

                    //modify this agv's taskgroup
                    pAGV2->m_pTaskGroup->SetTempTask_OnceDodge(this, pDodgePath, DodgePos[0], pAGV2->m_pTaskGroup->GetCurrentTask()->GetExecuteCombPath(),pAGV1->GetName());
                    //ApplyAgent occupy ApplyPath in advance
                    pPath2->m_bNextOccupyLock.OccupyLock(pAGV1);
                }
                else
                {
                    Path*   pTarget2Path = DodgeAgentApply.pCombPath->m_pPath;

                    //dodge path must not be the apply agent's current occupy path, not be the apply path, and  could not be lock in advance
                    bool   bcondition_0 = false;
                    vector<Path*>     PossiblePathList;
                    for(int i = 0; i < m_pPaths.size(); i++)
                    {
                        if(m_pPaths[i] != NULL)
                        {
                            Agent*  pAgnet = NULL;
                            m_pPaths[i]->m_bNextOccupyLock.GetOccupySub( pAgnet );

                            if( (m_pPaths[i] != pPath1) && (m_pPaths[i] != pPath2) &&
                               !(m_pPaths[i]->m_bNextOccupyLock.CheckLock())  &&
                                (m_pPaths[i]->m_bOccupyLock.CheckLock())      &&
                                (m_pPaths[i]->m_AGVOccupyList.size() == 1 )  )
                                PossiblePathList.push_back(m_pPaths[i]);
                        }
                    }
                    if(PossiblePathList.size() > 0)
                        bcondition_0 = true;
                    else
                        return;


                    /*************************************************************************/
                    /* Dodge path find logic                                                 */
                    /*                                                                       */
                    /* AGV1:  processing agent                                               */
                    /* AGV2:  dodge agent                                                    */
                    /* AGV3:  the agent on the dodge path                                    */
                    /*                                                                       */
                    /*                                                                       */
                    /* bcondition_3:   dodge path could park two agv                         */
                    /* bcondition_2:   dodge path is not the dodge agv's target              */
                    /* bcondition_1:   dodge path's direction is away from this across       */
                    /*                                                                       */
                    /* bcondition_3  NO    bcondition_1  YES                       case   5  */
                    /* bcondition_3  NO    bcondition_1  NO                        case   6  */
                    /* bcondition_3  YES   bcondition_2  YES                       case   1  */
                    /* bcondition_3  YES   bcondition_2  NO    bcondition_1  YES   case   3  */
                    /* bcondition_3  YES   bcondition_2  NO    bcondition_1  NO    case   2  */
                    /*************************************************************************/
                    int   dCase = 0;    //1  normal dodge, but dodge once
                                        //2  normal dodge, but need dodge twice
                                        //3  just wait
                                        //4  the worst case , the dodge path is too short to park two agents, need to deal in  some case 5 or 6
                                        //5  this dodge path's direction is away this across; just wait this dodge path's agent go away
                                        //6  this dodge path's direction is point to this across; rise priority of agent that on this path


                    //the dodge path's length must be longer than two agents length's sum
                    bool   bcondition_3 = false;
                    bool   bcondition_2 = false;
                    bool   bcondition_1 = false;
                    vector<Path*>     PossiblePathList_Cond3;
                    for(int i=0; i < PossiblePathList.size(); i++)
                    {                      
                        Path*               pPath3 = PossiblePathList[i];
                        Agent*              pAGV3  = PossiblePathList[i]->m_AGVOccupyList[0];
                        CombPath* pAGV3CurCombPath = pAGV3->m_pTaskGroup->GetCurrentTask()->GetExecuteCombPath();

                        std::vector<int>      dDodgeDirection;
                        std::vector<Vec2i>    DodgePos;
                        std::vector<Agent*>   pAGVList;
                        std::vector<int>      PostureList;

                        int dAGV2Direction = ( m_dID == pPath3->m_dAcrossID[0] ) ? 1 : 0;
                        int dAGV3Direction = pAGV3CurCombPath->m_dDirection;
                        dDodgeDirection.push_back(dAGV3Direction);
                        dDodgeDirection.push_back(dAGV2Direction);

                        Vec2i   AGV3Pos = pAGV3->GetCurrentGridPosition();
                        DodgePos.push_back( AGV3Pos );

                        pAGVList.push_back(pAGV3);
                        pAGVList.push_back(pAGV2);

                        int   AGV2Posture  = pAGV2->CalcAgentNextPosture( (Vec2i)*(this) );
                        Vec2i AGV3CurPos   = pAGV3->GetCurrentGridPosition();
                        Vec2i AGV3NexPos;
                        int   AGV3CurPosID = pPath3->GetSpeGridPosIndexWithDirection(AGV3CurPos, dAGV3Direction);
                        if( AGV3CurPosID < (pPath3->GetPathSize()-1) )
                            AGV3NexPos = pPath3->GetGridPosFromIndexDirection((AGV3CurPosID +1), dAGV3Direction);
                        else
                        {
                            //away this across
                            if( pAGV3CurCombPath->m_pPath->m_dAcrossID[pAGV3CurCombPath->m_dDirection] != m_dID )
                            {
                                int  dAcrossID = pAGV3CurCombPath->m_pPath->m_dAcrossID[pAGV3CurCombPath->m_dDirection];
                                AGV3NexPos = (Vec2i)*(m_pMap->m_AcrossPointList[dAcrossID]);
                            }
                            else   //toward this across
                                AGV3NexPos = (Vec2i)*(pAGV3CurCombPath->m_pAcross);
                        }
                        int   AGV3Posture  = pAGV2->CalcAgentNextPosture(AGV3NexPos);
                        PostureList.push_back(AGV3Posture);
                        PostureList.push_back(AGV2Posture);
                        bool  bLengthEnough = pPath3->GetDodgePos( dDodgeDirection, DodgePos, pAGVList, PostureList);

                        if( bLengthEnough == true )
                            PossiblePathList_Cond3.push_back( pPath3 );

                    }
                    if(PossiblePathList_Cond3.size() > 0)
                        bcondition_3 = true;
                    else
                    {
                        dCase = 4;

                        vector<Path*>  PossiblePathList_Cond3_AwayDirection; //first choose the path which direction is away this across
                        for(int i =0; i < PossiblePathList.size(); i++)
                        {
                            if(
                                 !(PossiblePathList[i]->m_bMonopoly) &&
                                  (PossiblePathList[i]->m_dAcrossID[PossiblePathList[i]->m_dDirection] != m_dID)
                                )
                                PossiblePathList_Cond3_AwayDirection.push_back(PossiblePathList[i]);
                        }
                        if(PossiblePathList_Cond3_AwayDirection.size() > 0)
                        {
                            dCase = 5;
                            Path*   pDodgePath = PossiblePathList_Cond3_AwayDirection[0];
                            //lock path, then wait
                            pDodgePath->m_bNextOccupyLock.OccupyLock(pAGV2);
                            pPath2->m_bNextOccupyLock.OccupyLock(pAGV1);
                        }
                        else
                        {
                            //the worst case,just change this path's prority
                            dCase = 6;
                            PossiblePathList[0]->m_bCouldUsedForDodge = false;
                        }
                    }

                    if( bcondition_3 )
                    {

                        vector<Path*>     PossiblePathList_Cond2;
                        for( int i =0 ; i < PossiblePathList_Cond3.size(); i++ )
                        {
                            if(PossiblePathList_Cond3[i] != pTarget2Path)
                                PossiblePathList_Cond2.push_back(PossiblePathList_Cond3[i]);
                        }

                        if(PossiblePathList_Cond2.size() > 0)
                        {
                            bcondition_2 = true;
                            dCase = 1;
                            dealMidApply_ConflictCase1(PossiblePathList_Cond2, pAGV1, pAGV2, pPath1, pPath2, UndealApplyList[0], DodgeAgentApply);
                        }

                        if(!bcondition_2)
                        {
                            //now PossiblePathList_Cond3's size must be 1
                            Path*   pDodgePath = PossiblePathList_Cond3[0];
                            if(
                                 (pDodgePath->m_bMonopoly == true)  ||
                                ((pDodgePath->m_bMonopoly == false) && (pDodgePath->m_dAcrossID[pDodgePath->m_dDirection]) == m_dID  )
                              )
                            {
                                bcondition_1 = true;
                                dCase        = 2;
                                dealMidApply_ConflictCase2( pAGV1,  pAGV2, pPath1,  pPath2,  pDodgePath );
                            }
                            else
                            {
                                dCase = 3;
                                //lock these two paths, and wait
                                pDodgePath->m_bNextOccupyLock.OccupyLock(pAGV2);
                                pPath2->m_bNextOccupyLock.OccupyLock(pAGV1);
                            }
                        }
                    }
                }
            }
        }
    }
    else if(  ( m_ApplyProcessing.pAgent != NULL ) && (m_ApplyProcessing.pCombPath != NULL)  && !CheckLock())
        dealMidApply_ApplyPass( m_ApplyProcessing );

}

void   AcrossPoint::dealMidApply_ApplyPass( ApplyInfor  &pProcessingApply )
{
    CombPath     *pCombPath = pProcessingApply.pCombPath;
    CombPathStatue  eStatue = pCombPath->GetStatue();
    string         sAGVName = pProcessingApply.pAgent->GetName();

    if(COMBPATH_MOVE_PATH <=  eStatue)
    {
        //when Agent1 pass across, then Agent2 will finished dodge mode
        for(int i =0; i < 8 ; i++)
        {
            Path* pPath = m_pPaths[i];
            if( pPath )
            {
                for(int j =0; j < pPath->m_AGVOccupyList.size();  j++)
                {
                    Agent* pAGV   = pPath->m_AGVOccupyList[j];

                    if(pAGV->m_pTaskGroup != NULL)
                    {
                        if(pAGV->m_pTaskGroup->m_TempTaskList.size() > 0)
                        {
                            for(int k =0; k < pAGV->m_pTaskGroup->m_TempTaskList.size(); k++)
                            {
                                Task*  pTask = pAGV->m_pTaskGroup->m_TempTaskList[k];
                                if( (pTask != NULL) && (pTask->m_ActionList.size() == 1) )
                                {
                                    bool   bCond_1 = (PT_DODGE == pTask->m_ActionList[0].m_eProType);
                                    bool   bCond_2 = false;

                                    string sReserveString = pTask->m_ActionList[0].m_sReserveString;
                                    int      OccurID  = sReserveString.find(" ");
                                    if(OccurID != -1)
                                    {
                                        string  AGVLockName  = sReserveString.substr(0, OccurID);
                                        if( AGVLockName == sAGVName )
                                        {
                                            sReserveString = sReserveString.substr(OccurID+1,sReserveString.size());
                                            pTask->m_ActionList[0].m_sReserveString = sReserveString;

                                            if( sReserveString.empty() )
                                                bCond_2 = true;
                                        }

                                    }

                                    if( bCond_1 && bCond_2 )
                                        pTask->m_ActionList[0].SetVarPara(true);
                                }
                            }
                        }
                    }

                }
            }
        }

        //if some path's next lock is locked, but occupy lock is empty,then would change next lock to occupy lock
        for(int i =0; i < 8 ; i++)
        {
            Path* pPath = m_pPaths[i];
            if( pPath )
            {
                bool  bNextLock = pPath->m_bNextOccupyLock.CheckLock();
                bool  bOccuLock = pPath->m_bOccupyLock.CheckLock();

                if(bNextLock && !bOccuLock)
                {
                    Agent*  pAGVNextLock = NULL;
                    pPath->m_bNextOccupyLock.GetOccupySub(pAGVNextLock);

                    //if( ( pPath->m_AGVOccupyList.size() == 1 ) && (pPath->IfAGVAlreadyOnThisPath(pAGVNextLock)) )
                    if(  pPath->m_AGVOccupyList.size() == 0 )
                    {
                        pPath->m_bNextOccupyLock.ReleaseLock();
                        pPath->m_bOccupyLock.OccupyLock(pAGVNextLock);
                    }
                }
            }
        }

        pProcessingApply.pAgent    = NULL;
        pProcessingApply.pCombPath = NULL;
    }
}

void   AcrossPoint::StaticPathNum()
{
    int  dPathNum = 0;
    for(int  i =0; i < m_pPaths.size(); i++)
    {
        if(m_pPaths[i] != NULL)
        {
            dPathNum++;
        }
    }

    m_dPathNum  = dPathNum;
}

void   AcrossPoint::GetAllNoUsedPath(std::vector<Path*> &pathlist)
{
    for(int i =0; i < m_pPaths.size(); i++)
    {
        Path* tempPath = m_pPaths[i];
        if(tempPath)
        {
            if(
                    !(tempPath->m_bOccupyLock.CheckLock())       &&
                    !(tempPath->m_bNextOccupyLock.CheckLock())   &&
                    (tempPath->m_AGVOccupyList.size() == 0)      &&
                    (tempPath->GetPathSize() > 0)                &&
                    ( -1 == tempPath->m_dDirection )
               )
                pathlist.push_back(tempPath);
        }
    }
}

int   AcrossPoint::GetPathNumberOfThisZone( )
{
    int  dPathNumber = 0;
    for(int i =0; i< 8; i++)
    {
        Path* pHead = m_pPaths[i];
        if( pHead != NULL )
            dPathNumber += 1;
    }

    return dPathNumber;
}

int   AcrossPoint::GetAGVNumberOfThisZone( )
{
    int  dAGVNumber = 0;
    for(int i =0; i< 8; i++)
    {
        Path* pHead = m_pPaths[i];
        if( pHead != NULL )
            dAGVNumber += pHead->m_AGVOccupyList.size();
    }

    return dAGVNumber;
}
