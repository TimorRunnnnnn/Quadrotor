
#include "Algorithm_quaternion.h"
#include "arm_math.h"
#include "math.h"
#include "ahrs.h"


/*====================================================================================================*/
/*====================================================================================================*
**���� : Quaternion_vectorGravity
**���� : ��ȡ��Ч��ת�����е���������
**���� : *pNumQ
**��� : ��������
**ʹ�� : Quaternion_vectorGravity( Quaternion *pNumQ );
**====================================================================================================*/
/*====================================================================================================*/
Gravity Quaternion_vectorGravity(Quaternion *pNumQ) {
	Gravity G;
	G.x = 2 * (pNumQ->q1*pNumQ->q3 - pNumQ->q0*pNumQ->q2);
	G.y = 2 * (pNumQ->q0*pNumQ->q1 + pNumQ->q2*pNumQ->q3);
	G.z = 1 - 2 * (pNumQ->q1*pNumQ->q1 + pNumQ->q2*pNumQ->q2);

	return G;
}
/*====================================================================================================*/
/*====================================================================================================*
**���� : Quaternion_ToNumQ
**���� : ŷ����ת��Ԫ��
**���� : *pNumQ, *pAngE
**��� : None
**ʹ�� : Quaternion_ToNumQ(&NumQ, &AngE);
**====================================================================================================*/
/*====================================================================================================*/
void Quaternion_ToNumQ(Quaternion *pNumQ, EulerAngle *pAngE) {
	float halfP = pAngE->Pitch / 2.0f;
	float halfR = pAngE->Roll / 2.0f;
	float halfY = pAngE->Yaw / 2.0f;

	float sinP = arm_sin_f32(halfP);
	float cosP = arm_cos_f32(halfP);
	float sinR = arm_sin_f32(halfR);
	float cosR = arm_cos_f32(halfR);
	float sinY = arm_sin_f32(halfY);
	float cosY = arm_cos_f32(halfY);

	pNumQ->q0 = cosY*cosR*cosP + sinY*sinR*sinP;
	pNumQ->q1 = cosY*cosR*sinP - sinY*sinR*cosP;
	pNumQ->q2 = cosY*sinR*cosP + sinY*cosR*sinP;
	pNumQ->q3 = sinY*cosR*cosP - cosY*sinR*sinP;
}
/*====================================================================================================*/
/*====================================================================================================*
**���� : Quaternion_ToAngE
**���� : ��Ԫ��תŷ����
**���� : *pNumQ, *pAngE
**��� : None
**ʹ�� : Quaternion_ToAngE(&NumQ, &AngE);
**====================================================================================================*/
/*====================================================================================================*/
void Quaternion_ToAngE(Quaternion *pNumQ, EulerAngle *pAngE) {
	float NumQ_T11 = pNumQ->q0*pNumQ->q0 + pNumQ->q1*pNumQ->q1 - pNumQ->q2*pNumQ->q2 - pNumQ->q3*pNumQ->q3;
	float NumQ_T12 = 2.0f*(pNumQ->q0*pNumQ->q3 + pNumQ->q1*pNumQ->q2);
	float NumQ_T13 = 2.0f*(pNumQ->q1*pNumQ->q3 - pNumQ->q0*pNumQ->q2);
	float NumQ_T23 = 2.0f*(pNumQ->q0*pNumQ->q1 + pNumQ->q2*pNumQ->q3);
	float NumQ_T33 = pNumQ->q0*pNumQ->q0 - pNumQ->q1*pNumQ->q1 - pNumQ->q2*pNumQ->q2 + pNumQ->q3*pNumQ->q3;

	pAngE->Pitch = -asinf(NumQ_T13);
	pAngE->Roll = atan2f(NumQ_T23, NumQ_T33);
	if(SensorStatue.magIssue==1)
	pAngE->Yaw = atan2f(NumQ_T12, NumQ_T11);
}
/*====================================================================================================*/
/*====================================================================================================*
**���� : Quaternion_Multiply
**���� : ��Ԫ���˷�
**���� : NowQ, OldQ
**��� : NewQ
**ʹ�� : NewQ = Quaternion_Multiply(NowQ, OldQ);
**====================================================================================================*/
/*====================================================================================================*/
Quaternion Quaternion_Multiply(Quaternion NowQ, Quaternion OldQ) {
	Quaternion NewQ;

	NewQ.q0 = NowQ.q0*OldQ.q0 - NowQ.q1*OldQ.q1 - NowQ.q2*OldQ.q2 - NowQ.q3*OldQ.q3;
	NewQ.q1 = NowQ.q0*OldQ.q1 + NowQ.q1*OldQ.q0 + NowQ.q2*OldQ.q3 - NowQ.q3*OldQ.q2;
	NewQ.q2 = NowQ.q0*OldQ.q2 - NowQ.q1*OldQ.q3 + NowQ.q2*OldQ.q0 + NowQ.q3*OldQ.q1;
	NewQ.q3 = NowQ.q0*OldQ.q3 + NowQ.q1*OldQ.q2 - NowQ.q2*OldQ.q1 + NowQ.q3*OldQ.q0;

	Quaternion_Normalize_(&NewQ);

	return NewQ;
}
/*====================================================================================================*/
/*====================================================================================================*
**���� : Quaternion_Normalize
**���� : ��Ԫ����һ��
**���� : *pNumQ
**��� : None
**ʹ�� : Quaternion_Normalize(&NewQ);
**====================================================================================================*/
/*====================================================================================================*/
void Quaternion_Normalize_(Quaternion *pNumQ) {
	float Normalize = 0.0f;

	Normalize = Q_rsqrt(squa(pNumQ->q0) + squa(pNumQ->q1) + squa(pNumQ->q2) + squa(pNumQ->q3));

	pNumQ->q0 = pNumQ->q0 * Normalize;
	pNumQ->q1 = pNumQ->q1 * Normalize;
	pNumQ->q2 = pNumQ->q2 * Normalize;
	pNumQ->q3 = pNumQ->q3 * Normalize;
}
/*====================================================================================================*/
/*====================================================================================================*
**���� : Quaternion_RungeKutta
**���� : һ�����������, ������Ԫ��
**���� : *pNumQ, GyrX, GyrY, GyrZ, helfTimes
**��� : None
**ʹ�� : Quaternion_RungeKutta(&NumQ, GyrX, GyrY, GyrZ, helfT);
**====================================================================================================*/
/*====================================================================================================*/
void Quaternion_RungeKutta(Quaternion *pNumQ, float GyrX, float GyrY, float GyrZ, float helfTimes) {
	float tmpq0 = pNumQ->q0;
	float tmpq1 = pNumQ->q1;
	float tmpq2 = pNumQ->q2;
	float tmpq3 = pNumQ->q3;

	pNumQ->q0 = pNumQ->q0 + (-tmpq1*GyrX - tmpq2*GyrY - tmpq3*GyrZ) * helfTimes;
	pNumQ->q1 = pNumQ->q1 + (tmpq0*GyrX - tmpq3*GyrY + tmpq2*GyrZ) * helfTimes;
	pNumQ->q2 = pNumQ->q2 + (tmpq3*GyrX + tmpq0*GyrY - tmpq1*GyrZ) * helfTimes;
	pNumQ->q3 = pNumQ->q3 + (-tmpq2*GyrX + tmpq1*GyrY + tmpq0*GyrZ) * helfTimes;
}
/*====================================================================================================*/

