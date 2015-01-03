#ifndef _DEFINITIONS_H_
#define _DEFINITIONS_H_

typedef union _taddress taddress;

union _taddress {
	struct {
		unsigned char l;
		unsigned char h;
	} b;
	unsigned short w;

  inline bool operator>(const taddress& r){
    return this->w > r.w;
  }
  inline bool operator>(const unsigned short& r){
    return this->w > r;
  }
  inline bool operator<(const taddress& r){
    return this->w < r.w;
  }
  inline bool operator<(const unsigned short& r){
    return this->w < r;
  }
  inline bool operator>=(const taddress& r){
    return this->w >= r.w;
  }
  inline bool operator>=(const unsigned short& r){
    return this->w >= r;
  }
  inline bool operator<=(const taddress& r){
    return this->w <= r.w;
  }
  inline bool operator<=(const unsigned short& r){
    return this->w <= r;
  }
  inline taddress operator-(unsigned short w){
    taddress _new(*this);
    _new.w-=w;
    return _new;
  }
  inline taddress operator+(unsigned short w){
    taddress _new(*this);
    _new.w+=w;
    return _new;
  }
  inline taddress operator=(unsigned short w){
    this->w = w;
    return *this;
  }
  inline taddress operator+=(unsigned short w){
    this->w += w;
    return *this;
  }
  inline taddress operator-=(unsigned short w){
    this->w -= w;
    return *this;
  }
  inline taddress& operator++(){
    this->w++;
    return *this;
  }
  inline taddress operator++(int){
    taddress tmp(*this);
    operator++();
    return tmp;
  }
  inline operator unsigned short(){
    return this->w;
  }
};


#endif
