#pragma once

#include "stackObject.h"

class CStackString : public CStackObject
{
public:
	CStackString(const char *str, int l);

	virtual ~CStackString();

	CStackObject *copyYourself();

	std::string getValue();

	void setValue(const char *str, int l);

protected:
	std::string _value;
};
