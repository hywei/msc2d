// HSVColor.h: interface for the CHSVColor class.
//
//////////////////////////////////////////////////////////////////////

#ifndef CHSVCOLOR
#define CHSVCOLOR

class CHSVColor  
{
public:
	float minofthree(float a, float b, float c);
	float maxofthree(float a, float b, float c);
	void HSVtoRGB(float *R, float *G, float *B);
	void RGBtoHSV(float R, float G, float B);
	float m_V;
	float m_S;
	float m_H;
	CHSVColor();
	virtual ~CHSVColor();

};

#endif
