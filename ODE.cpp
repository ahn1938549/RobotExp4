#include "stdafx.h"
#include "ODE.h"
#include "SystemMemory.h"
#include "DataType.h"

#ifndef DRAWSTUFF_TEXTURE_PATH
#define DRAWSTUFF_TEXTURE_PATH "./../ode-0.13/drawstuff/textures"
#endif

#define GRAVITY 9.81
#define MAX_JOINT_NUM 2

#define DEG2RAD 0.0174533
#define RAD2DEG 57.2958

dsFunctions g_Fn;

static dWorldID g_World;
static dSpaceID g_Space;
static dJointGroupID g_Contactgroup;

Object g_oObj[MAX_JOINT_NUM + 1];
static dJointID g_oJoint[MAX_JOINT_NUM + 1];

double g_tar_q[MAX_JOINT_NUM] = { 0.0, 0.0 };
double g_cur_q[MAX_JOINT_NUM] = { 0.0, 0.0};

void InitDrawStuff() {

	g_Fn.version = DS_VERSION;
	g_Fn.start = &StartDrawStuff;
	g_Fn.step = &SimLoopDrawStuff;
	g_Fn.command = &CommandDrawStuff;
	g_Fn.stop = StopDrawStuff;
	g_Fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
}

//#dGeomID g_Ground;
void InitODE() { // 중력등 초기설정 

	dInitODE();
	g_World = dWorldCreate();
	g_Space = dHashSpaceCreate(0);
	g_Contactgroup = dJointGroupCreate(0);
	dWorldSetGravity(g_World, 0, 0, -GRAVITY);
    //#g_Ground = dCreatePlane(g_Space, 0, 0, 1, 0);
	dWorldSetCFM(g_World, 1e-5);
	
}



void RunODE(size_t width, size_t height) {

	//TO DO
	InitDrawStuff();
	InitODE();

	InitRobot();
	//simulation Part
	dsSimulationLoop(0, 0, width, height, &g_Fn);
}



void ReleaseODE() {

	dJointGroupDestroy(g_Contactgroup);
	dSpaceDestroy(g_Space);
	dWorldDestroy(g_World);
	dCloseODE();
}



void StartDrawStuff() {

	//TO DO

}


void SimLoopDrawStuff(int pause) 
{

	//TO DO
	//루프에 바디를 그리는 함수를 집어넣는 과정
	dReal dR, dLength;

	dsSetColor(0, 1, 0);
	dGeomCapsuleGetParams(g_oObj[0].geom, &dR, &dLength);
	dsDrawCapsuleD(dBodyGetPosition(g_oObj[0].body), dBodyGetRotation(g_oObj[0].body), (float)dLength, (float)dR);

	dsSetColor(0, 0, 1);
	dGeomCapsuleGetParams(g_oObj[1].geom, &dR, &dLength);
	dsDrawCapsuleD(dBodyGetPosition(g_oObj[1].body), dBodyGetRotation(g_oObj[1].body), (float)dLength, (float)dR);

	

	double dt = 0.01;
	dWorldStep(g_World, dt);
	
}



void CommandDrawStuff(int cmd) {

	//TO DO

}



void StopDrawStuff() {

	//TO DO

}


void InitRobot()
{

	//TO DO

	dMass mass;
	dMatrix3 R;

	dReal x[MAX_JOINT_NUM] = { 0.0 , 0.00 };
	dReal y[MAX_JOINT_NUM] = { 0.0 , 0.00 };
	dReal z[MAX_JOINT_NUM] = { 0.5 , 1.25 }; // 질량 중싱점

	dReal ori_x[MAX_JOINT_NUM] = { 0.0 , 0.0 };
	dReal ori_y[MAX_JOINT_NUM] = { 0.0 , 0.0 };
	dReal ori_z[MAX_JOINT_NUM] = { 1.0 , 1.0 };
	dReal ori_q[MAX_JOINT_NUM] = { 0.0 , 0.0 }; //링크 자세

	dReal length[MAX_JOINT_NUM] = { 1.0 , 0.5 };
	dReal weight[MAX_JOINT_NUM] = { 1.0 , 1.0 };
	dReal r[MAX_JOINT_NUM] = { 0.1f , 0.1f };

	//body 생성
	for (int i = 0; i < MAX_JOINT_NUM; i++) {
		g_oObj[i].body = dBodyCreate(g_World);
		dBodySetPosition(g_oObj[i].body, x[i], y[i], z[i]);
		dMassSetZero(&mass);
		dMassSetCapsuleTotal(&mass, weight[i], 1, r[i], length[i]);
		dBodySetMass(g_oObj[i].body, &mass);//물체의 위치 생성

		g_oObj[i].geom = dCreateCapsule(g_Space, r[i], length[i]);
		dGeomSetBody(g_oObj[i].geom, g_oObj[i].body);
		dRFromAxisAndAngle(R, ori_x[i], ori_y[i], ori_z[i], ori_q[i]);
		dBodySetRotation(g_oObj[i].body, R);//물체의 형태 방향 등 설정
	}
		dReal djointx[MAX_JOINT_NUM] = { 0.0 , 0.0 };
		dReal djointy[MAX_JOINT_NUM] = { 0.0 , 0.0 };
		dReal djointz[MAX_JOINT_NUM] = { 0.0 , 1.0 };

		dReal axis_x[MAX_JOINT_NUM] = { 0.0 , 0.0 };
		dReal axis_y[MAX_JOINT_NUM] = { 0.0 , 1.0 };
		dReal axis_z[MAX_JOINT_NUM] = { 1.0 , 0.0 };

		//joint 결합
		//먼저 fixed joint 부터 world와 연결
		g_oJoint[0] = dJointCreateFixed(g_World, 0);
		dJointAttach(g_oJoint[0], g_oObj[0].body, 0);
		dJointSetFixed(g_oJoint[0]);
		//Hinge Joint생성 및 결합
		for(int i =1; i <MAX_JOINT_NUM; i++) {
			g_oJoint[i] = dJointCreateHinge(g_World, 0);
			dJointAttach(g_oJoint[i], g_oObj[i].body, g_oObj[i - 1].body);
			dJointSetHingeAnchor(g_oJoint[i], djointx[i], djointy[i], djointz[i]);
			dJointSetHingeAxis(g_oJoint[i], axis_x[i], axis_y[i], axis_z[i]);
		

	}
}

void PControl()
{
	/*
	//TO DO

	dReal Kp = 10; , fMax = 5000.0;
	dReal a_error_q[MAX_JOINT_NUM];

	for (int i = 1; i < MAX_JOINT_NUM; i++) 
	{
		m_cur_q[i] = dJointGetHingeAngle(capsuleJoint[i]);
		a_error_q[i] = m_tar_q[i] - m_cur_q[i];
		dJointSetHingeParam(capsuleJoint[i].dParamVel.Kp * a_error_q[i]);
		dJointSetHingeParam(capsuleJoint[i].dParamFMax.famx);
	}
	*/
}
