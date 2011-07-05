// HSVColor.cpp: implementation of the CHSVColor class.
//
//////////////////////////////////////////////////////////////////////
#include "hsvcolor.h"
#include <math.h>

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CHSVColor::CHSVColor()
{

}

CHSVColor::~CHSVColor()
{

}

void CHSVColor::RGBtoHSV(float R, float G, float B)
{
	float MaxValue, MinValue, diff, Rdist, Gdist, Bdist;

	MaxValue = maxofthree(R, G, B);//
	MinValue = minofthree(R, G, B);//
	diff = MaxValue - MinValue;
	m_V = MaxValue;
	if(MaxValue !=0)
		m_S = diff/MaxValue;
	else {
		m_S = 0;
		m_H = 0;
	}
	if(m_S!=0)
	{
		Rdist = (MaxValue - R)/diff;
		Gdist = (MaxValue - G)/diff;
		Bdist = (MaxValue - B)/diff;
		if(R == MaxValue)
			m_H = Bdist - Gdist;
		else if(G == MaxValue)
			m_H = 2+Rdist-Bdist;
		else if(B == MaxValue)
			m_H = 4+Gdist-Rdist;

		m_H = m_H*60;
		if(m_H<0)
			m_H += 360;
	}
}

void CHSVColor::HSVtoRGB(float *R, float *G, float *B)
{
	float f, p, q, t;
	int i;
	float tmpH = m_H;
	float tmpS = m_S;
	float tmpV = m_V;
	if(tmpS == 0)
	{
		*R = tmpV;
		*G = tmpV;
		*B = tmpV;
	}
	else {
		if(tmpH == 360)
		{
			tmpH = 0;
			m_H = 0;
		}
		tmpH /= 60.0f;
		i = (int)floor(tmpH);//
		f = tmpH-i;
		p = tmpV * (1 - tmpS);
		q = tmpV * (1 - (tmpS * f));
		t = tmpV * (1 - (tmpS * (1 - f)));
		switch(i){
		case 0:
			*R = tmpV;
			*G = t;
			*B = p;
			break;
		case 1:
			*R = q;
			*G = tmpV;
			*B = p;
			break;
		case 2:
			*R = p;
			*G = tmpV;
			*B = t;
			break;
		case 3:
			*R = p;
			*G = q;
			*B = tmpV;
			break;
		case 4:
			*R = t;
			*G = p;
			*B = tmpV;
			break;
		case 5:
			*R = tmpV;
			*G = p;
			*B = q;
			break;
		default:
			break;
		}
	}

}

float CHSVColor::maxofthree(float a, float b, float c)
{
	float max;
	if(a>b)
	{
		if(a>c)
			max = a;
		else max = c;
	}
	else {
		if(b>c)
			max = b;
		else max = c;
	}
	return max;
}

float CHSVColor::minofthree(float a, float b, float c)
{
	float min;
	if(a<b)
	{
		if(a<c)
			min = a;
		else min = c;
	}
	else {
		if(b<c)
			min = b;
		else min = c;
	}
	return min;
}
