#include <vrep/Helper.h>

Helper::Helper(const unsigned int &dim) {
    m_dim = dim;
    m_clientId = -1;

    m_jointHandles = Vec<simxInt>(m_dim);
    m_linkHandles = Vec<simxInt>(m_dim);
    m_collisionHandle = Vec<simxInt>(m_dim);
    for (unsigned int i = 0; i < m_dim; ++i)
        m_jointHandles[i] = -1;
}

void Helper::start() {
    std::cout << "Program started" << std::endl;

    simxInt errorCodejaco, errorcode[10];

    m_clientId=simxStart((simxChar*)"127.0.0.1",19999,true,true,2000,5);
    if (m_clientId!=-1) {
        std::cout << "connected to  remote API" << std::endl;

        errorcode[1]=simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_joint1",&m_jointHandles[0],simx_opmode_oneshot_wait);
        errorcode[2]=simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_joint2",&m_jointHandles[1],simx_opmode_oneshot_wait);
        errorcode[3]=simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_joint3",&m_jointHandles[2],simx_opmode_oneshot_wait);
        errorcode[4]=simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_joint4",&m_jointHandles[3],simx_opmode_oneshot_wait);
        errorcode[5]=simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_joint5",&m_jointHandles[4],simx_opmode_oneshot_wait);
        errorcode[6]=simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_joint6",&m_jointHandles[5],simx_opmode_oneshot_wait);
        simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_link1",&m_linkHandles[0],simx_opmode_oneshot_wait);
        simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_link2",&m_linkHandles[1],simx_opmode_oneshot_wait);
        simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_link3",&m_linkHandles[2],simx_opmode_oneshot_wait);
        simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_link4",&m_linkHandles[3],simx_opmode_oneshot_wait);
        simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_link5",&m_linkHandles[4],simx_opmode_oneshot_wait);
        simxGetObjectHandle(m_clientId,(simxChar*)"Jaco_link6",&m_linkHandles[5],simx_opmode_oneshot_wait);
        m_collisionHandle[0] = simxGetCollisionHandle(m_clientId, (simxChar*)"Jaco_link1", &m_linkHandles[0], simx_opmode_oneshot_wait);
        m_collisionHandle[1] = simxGetCollisionHandle(m_clientId, (simxChar*)"Jaco_link2", &m_linkHandles[1], simx_opmode_oneshot_wait);
        m_collisionHandle[2] = simxGetCollisionHandle(m_clientId, (simxChar*)"Jaco_link3", &m_linkHandles[2], simx_opmode_oneshot_wait);
        m_collisionHandle[3] = simxGetCollisionHandle(m_clientId, (simxChar*)"Jaco_link4", &m_linkHandles[3], simx_opmode_oneshot_wait);
        m_collisionHandle[4] = simxGetCollisionHandle(m_clientId, (simxChar*)"Jaco_link5", &m_linkHandles[4], simx_opmode_oneshot_wait);
        m_collisionHandle[5] = simxGetCollisionHandle(m_clientId, (simxChar*)"Jaco_link6", &m_linkHandles[5], simx_opmode_oneshot_wait);

        simxInt handleAll = -2;
        simxInt linkhandle = simxGetObjectHandle(m_clientId, (simxChar*)"Jaco_link1", &handleAll, simx_opmode_oneshot_wait);

        //for(int i=0;i<6;i++)
        //    printf("Joint code%d %d\n",i,m_jointHandles[i]);

        errorCodejaco=simxGetObjectHandle(m_clientId,(simxChar*)"Jaco", &m_jacoHandle , simx_opmode_oneshot_wait);
        printf("Error code Jaco %d\n",errorCodejaco);

        simxUChar collisionState;
        for (unsigned int i = 0; i < m_dim; ++i) {
            simxReadCollision(m_clientId, m_collisionHandle[i], &collisionState, simx_opmode_oneshot_wait);
            printf("Error code Jaco link collision %d %d\n",i,collisionState);
        }

        std::cout << std::endl << std::endl;
    }
    else {
        std::cout << "could not connect to server" << std::endl;
    }
}

bool Helper::setPos(const Vec<float> &vec) {
    if (m_clientId != -1) {
        Vec<simxFloat> pos = convertVecToRad(vec);
        for (unsigned int i = 0; i < m_dim; ++i)
            simxSetJointTargetPosition(m_clientId,m_jointHandles[i],pos[i],simx_opmode_oneshot_wait);
        usleep(3000000);
        int *distHandle;
        float *dist;
        simxGetObjectHandle(m_clientId, "distance", distHandle, simx_opmode_oneshot_wait);
        simxReadDistance(m_clientId, *distHandle, dist, simx_opmode_oneshot_wait);
        simxInt numDist;
        simxFloat *distance;
        simxCallScriptFunction(m_clientId,"Jaco",sim_scripttype_childscript,"checkDist_function",0,NULL,0,NULL,0,NULL,0,NULL,NULL,NULL,&numDist,&distance,NULL,NULL,NULL,NULL,simx_opmode_blocking);

        if (numDist == NULL)
            std::cout << "no output" << std::endl;
        else
            if (numDist > 0)
                std::cout << "distance: " << distance[0] << std::endl;
    }
}

bool Helper::isInCollision(const Vec<float> &jointAngles)
{
    int retJointColCnt;
	int* retJointCols;
	int result=0;
	bool flag;
	float startGoalPos[12]={0};
	float jointPosition[6]={0};
	for(uint16_t i=0;i<6;i++){
			simxGetJointPosition(m_clientId,m_jointHandles[i],&jointPosition[i],simx_opmode_oneshot_wait);
	}
	for(uint16_t i=0;i<6;i++){
		startGoalPos[i]=jointPosition[i];
		startGoalPos[i+6]=jointAngles[i];
	}
	int collisionJacoResult=simxCallScriptFunction(m_clientId,"Jaco",sim_scripttype_childscript,"checkCollisionOMPL_function",0,NULL,12,startGoalPos,0,NULL,0,NULL,&retJointColCnt,&retJointCols,NULL,NULL,NULL,NULL,NULL,NULL,simx_opmode_blocking);
	if(retJointCols[0]==1)
		flag=false;
	else
		flag=true;

	return flag;
}

bool Helper::isInCollision(const Vec<float> &qStart, const Vec<float> &qGoal)
{
    Vec<simxFloat> start = convertVecToRad(qStart);
    Vec<simxFloat> goal = convertVecToRad(qGoal);
	simxInt retJointColCnt;
	simxInt* retJointCols;
	simxInt result=0;
	bool flag;
	float startGoalPos[12];
	for(int i=0;i<6;i++){
		startGoalPos[i]=start[i];
		startGoalPos[i+6]=goal[i];
	}
	//simxInt collisionJacoResult=simxCallScriptFunction(m_clientId,"Jaco",sim_scripttype_childscript,"checkCollisionOMPL_function",(simxInt)6,m_jointHandles,(simxInt)12,startGoalPos,(simxInt)0,NULL,0,NULL,&retJointColCnt,&retJointCols,NULL,NULL,NULL,NULL,NULL,NULL,simx_opmode_blocking);
	if(retJointCols[0]==1)
		flag=false;
	else
		flag=true;

	return flag;
}

Vec<simxFloat> Helper::convertVecToRad(const Vec<float> &vec) {
    Vec<simxFloat> rads(vec.getDim());
    for (unsigned int i = 0; i < m_dim; ++i) {
        rads[i] = vec[i]*M_PI/180;
    }
    return rads;
}
