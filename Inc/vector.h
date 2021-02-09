#include <stdlib.h>
#include <math.h>
//#include "stm32f3xx_hal.h"

typedef float Vector3f[3];
typedef float Matrix3f[3][3];

//#define Pi 3.14159265358979323846f;
#define PiOver2 1.570796326794897f;

void M3fSetRow(Matrix3f M1, Vector3f V1, uint8_t Row)
{
	M1[Row][0]=V1[0];
	M1[Row][1]=V1[1];
	M1[Row][2]=V1[2];
}

void M3fMultiply(Matrix3f M1, Matrix3f M2, Matrix3f MRes)
{
	MRes[0][0]=M1[0][0]*M2[0][0] + M1[0][1]*M2[1][0] + M1[0][2]*M2[2][0];
	MRes[0][1]=M1[0][0]*M2[0][1] + M1[0][1]*M2[1][1] + M1[0][2]*M2[2][1];
	MRes[0][2]=M1[0][0]*M2[0][2] + M1[0][1]*M2[1][2] + M1[0][2]*M2[2][2];

	MRes[1][0]=M1[1][0]*M2[0][0] + M1[1][1]*M2[1][0] + M1[1][2]*M2[2][0];
	MRes[1][1]=M1[1][0]*M2[0][1] + M1[1][1]*M2[1][1] + M1[1][2]*M2[2][1];
	MRes[1][2]=M1[1][0]*M2[0][2] + M1[1][1]*M2[1][2] + M1[1][2]*M2[2][2];

	MRes[2][0]=M1[2][0]*M2[0][0] + M1[2][1]*M2[1][0] + M1[2][2]*M2[2][0];
	MRes[2][1]=M1[2][0]*M2[0][1] + M1[2][1]*M2[1][1] + M1[2][2]*M2[2][1];
	MRes[2][2]=M1[2][0]*M2[0][2] + M1[2][1]*M2[1][2] + M1[2][2]*M2[2][2];
}

void V3fTransform(Vector3f V, Matrix3f M, Vector3f VRes)
{
	VRes[0]=V[0]*M[0][0] + V[1]*M[1][0] + V[2]*M[2][0];
	VRes[1]=V[0]*M[0][1] + V[1]*M[1][1] + V[2]*M[2][1];
	VRes[2]=V[0]*M[0][2] + V[1]*M[1][2] + V[2]*M[2][2];
}

void V3fMult(Vector3f V, float K)
{
	V[0]=V[0]*K;
	V[1]=V[1]*K;
	V[2]=V[2]*K;
}

void V3fEMA(Vector3f V0, Vector3f V1, float K)
{
   V0[0]= V0[0]*(1-K) + V1[0]*K;
   V0[1]= V0[1]*(1-K) + V1[1]*K;
   V0[2]= V0[2]*(1-K) + V1[2]*K;
}


//------Дальше - на следующий стрим




float M3fDefiner(Matrix3f M1)
{
	float def;
	def=    M1[0][0] * M1[1][1] * M1[2][2]
		  + M1[1][0] * M1[0][2] * M1[2][1]
		  + M1[0][1] * M1[2][0] * M1[1][2]
		  - M1[2][0] * M1[1][1] * M1[0][2]
		  - M1[1][0] * M1[2][2] * M1[0][1]
		  - M1[2][1] * M1[0][0] * M1[1][2];
	return def;
}

int8_t M3fInvert(Matrix3f M1, Matrix3f MInv)
{
	float def = M3fDefiner(M1);
	if (def==0)
		return 0;
	else
		def=1.0f/def;

	MInv[0][0]=(M1[1][1]*M1[2][2]-M1[1][2]*M1[2][1])*def;
	MInv[1][0]=(-M1[1][0]*M1[2][2]+M1[1][2]*M1[2][0])*def;
	MInv[2][0]=(M1[1][0]*M1[2][1]-M1[1][1]*M1[2][0])*def;

	MInv[0][1]=(-M1[0][1]*M1[2][2]+M1[0][2]*M1[2][1])*def;
	MInv[1][1]=(M1[0][0]*M1[2][2]-M1[0][2]*M1[2][0])*def;
	MInv[2][1]=(-M1[0][0]*M1[2][1]+M1[0][1]*M1[2][0])*def;

	MInv[0][2]=(M1[0][1]*M1[1][2]-M1[0][2]*M1[1][1])*def;
	MInv[1][2]=(-M1[0][0]*M1[1][2]+M1[0][2]*M1[1][0])*def;
	MInv[2][2]=(M1[0][0]*M1[1][1]-M1[0][1]*M1[1][0])*def;
		return 1;
}

void M3fDupe(Matrix3f M1, Matrix3f MRes)
{
	MRes[0][0]=M1[0][0];
	MRes[0][1]=M1[0][1];
	MRes[0][2]=M1[0][2];

	MRes[1][0]=M1[1][0];
	MRes[1][1]=M1[1][1];
	MRes[1][2]=M1[1][2];

	MRes[2][0]=M1[2][0];
	MRes[2][1]=M1[2][1];
	MRes[2][2]=M1[2][2];
}

void V3fDupe(Vector3f V1, Vector3f VRes)
{
	VRes[0]=V1[0];
	VRes[1]=V1[1];
	VRes[2]=V1[2];
}

float V3fDot(Vector3f V1, Vector3f V2)
{
	return V1[0]*V2[0] + V1[1]*V2[1] + V1[2]*V2[2];
}

float V3fLength(Vector3f V1)
{
	return sqrt(V1[0]*V1[0] + V1[1]*V1[1] + V1[2]*V1[2]);
}

float V3fLengthSqr(Vector3f V1)
{
	return (V1[0]*V1[0] + V1[1]*V1[1] + V1[2]*V1[2]);
}

float V3fCos(Vector3f V1, Vector3f V2)
{
	return V3fDot(V1,V2)/sqrt(V3fLengthSqr(V1)*V3fLengthSqr(V2));
}

float V3fAng(Vector3f V1, Vector3f V2)
{
	return acos(V3fDot(V1,V2)/sqrt(V3fLengthSqr(V1)*V3fLengthSqr(V2)));
}


void V3fCross(Vector3f V1, Vector3f V2, Vector3f VRes)
{
	VRes[0]=V1[1]*V2[2] - V1[2]*V2[1];
	VRes[1]=V1[2]*V2[0] - V1[0]*V2[2];
	VRes[2]=V1[0]*V2[1] - V1[1]*V2[0];
}

void V3fNormalize(Vector3f V1, Vector3f VRes)
{
	float VLen;
	VLen=1.0f/V3fLength(V1);
	VRes[0]=V1[0]*VLen;
	VRes[1]=V1[1]*VLen;
	VRes[2]=V1[2]*VLen;
}

void V3fNormalizeSelf(Vector3f V1)
{
	float VLen;
	VLen=1.0f/V3fLength(V1);
	V1[0]=V1[0]*VLen;
	V1[1]=V1[1]*VLen;
	V1[2]=V1[2]*VLen;
}

void V3fProject(Vector3f V1, Vector3f V2, Vector3f VRes)
{
	float K;
	K=V3fDot(V1,V2)/V3fLengthSqr(V2);
	VRes[0]=V2[0]*K;
	VRes[1]=V2[1]*K;
	VRes[2]=V2[2]*K;
}

void V3fReject(Vector3f V1, Vector3f V2, Vector3f VRes)
{
	float K;
	K=V3fDot(V1,V2)/V3fLengthSqr(V2);
	VRes[0]=V1[0]-V2[0]*K;
	VRes[1]=V1[1]-V2[1]*K;
	VRes[2]=V1[2]-V2[2]*K;
}

void V3fReflect(Vector3f V1, Vector3f V2, Vector3f VRes)
{
	float K;
	K=V3fDot(V1,V2)/V3fLengthSqr(V2);
	VRes[0]=V2[0]*K*2.0f-V1[0];
	VRes[1]=V2[1]*K*2.0f-V1[1];
	VRes[2]=V2[2]*K*2.0f-V1[2];
}

void V3fReflectM(Vector3f V1, Vector3f V2, float M, Vector3f VRes)
{
	float K;
	K=V3fDot(V1,V2)/V3fLengthSqr(V2);
	VRes[0]=V2[0]*K*(1.0f+M)-V1[0]*M;
	VRes[1]=V2[1]*K*(1.0f+M)-V1[1]*M;
	VRes[2]=V2[2]*K*(1.0f+M)-V1[2]*M;
}

/*void V3fMult(Vector3f V1, float M, Vector3f VRes)
{
	VRes[0]=V1[0]*M;
	VRes[1]=V1[1]*M;
	VRes[2]=V1[2]*M;
}
*/

void V3fDiv(Vector3f V1, float M, Vector3f VRes)
{
	float InvM=1.0f/M;
	VRes[0]=V1[0]*InvM;
	VRes[1]=V1[1]*InvM;
	VRes[2]=V1[2]*InvM;
}

void V3fInvert(Vector3f V1, Vector3f VRes)
{
	VRes[0]=-V1[0];
	VRes[1]=-V1[1];
	VRes[2]=-V1[2];
}

void V3fInvertSelf(Vector3f V1)
{
	V1[0]=-V1[0];
	V1[1]=-V1[1];
	V1[2]=-V1[2];
}

void V3fSum(Vector3f V1, Vector3f V2, float M, Vector3f VRes)
{
	VRes[0]=V1[0]+V2[0];
	VRes[1]=V1[1]+V2[1];
	VRes[2]=V1[2]+V2[2];
}

void V3fDiff(Vector3f V1, Vector3f V2, float M, Vector3f VRes)
{
	VRes[0]=V1[0]-V2[0];
	VRes[1]=V1[1]-V2[1];
	VRes[2]=V1[2]-V2[2];
}


uint8_t M3fGetEuler(Matrix3f M1, Vector3f Euler)
{
	float fi = M1[0][2];
	if (fi < 1.0f)
	{
		if (fi > -1.0f)
		{
			Euler[0] = atan2(-M1[1][2], M1[2][2]);
			Euler[1] = asin(M1[0][2]);
			Euler[2] = atan2(-M1[0][1], M1[0][0]);
			return 1;
		}
		else
		{
			// WARNING.  Not unique.  XA - ZA = -atan2(r10,r11)
			Euler[0] = -atan2(M1[1][0], M1[1][1]);
			Euler[1] = -PiOver2;
			Euler[2] = 0.0f;
			return 0;
		}
	}
	else
	{
		    // WARNING.  Not unique.  XAngle + ZAngle = atan2(r10,r11)
			Euler[0] = -atan2(M1[1][0], M1[1][1]);
			Euler[1] = -PiOver2;
			Euler[2] = 0.0f;
	}
	return 0;
}

void floatToByteArray(float f, uint8_t *barr, uint8_t start) {

    unsigned int asInt = *((int*)&f);

    int i;
    for (i = 0; i < 4; i++) {
        barr[start+i] = (asInt >> 8 * i) & 0xFF;
    }
}
