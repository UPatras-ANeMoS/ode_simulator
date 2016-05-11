//
//  Symbolic.h
//  SymToODE
//
//  Created by Kostis Giannousakis on 26/01/16.
//  Copyright © 2016 Kostis Giannousakis. All rights reserved.
//

#ifndef Symbolic_h
#define Symbolic_h

#include "CoreTypesLite.h"

typedef enum _SMElementType {
    
    SMElementTypeUknown     = 0x00,
    SMElementTypeSymbol     = 0x01,
    SMElementTypeNumber     = 0x02,
    
    SMElementTypeGroup      = 0x03,
    SMElementTypePow        = 0x04,
    SMElementTypeMult       = 0x05,
    SMElementTypeDiv        = 0x06,
    SMElementTypeAdd        = 0x07,
    
    SMElementTypeMatrix     = 0x08,
    SMElementTypeSubMatrix  = 0x09,
    SMElementTypeFun        = 0x0A,
    
    SMElementTypeNegativeMask = 0xF0
    
} SMElementType;


typedef struct _SMElement {
    SMElementType   type;
} SMElement;

void SMElementDealloc(SMElement *obj);


#pragma mark - SMString

typedef struct _SMString {
    char  *data;
    size_t len;
    size_t size;
} SMString;


typedef struct _SMSubString {
    char  *data;
    size_t len;
} SMSubString;


SMString * SMStringCreate();
SMString * SMStringCreateWithString(const char *str);
SMString * SMStringCreateWithSubString(SMSubString str);
void SMStringDealloc(SMString *obj);

CTResult SMStringEmpty(SMString * obj);

CTResult SMStringLoadContentsOfFile(SMString * obj, const char *file);
CTResult SMStringAppendChar(SMString * obj, char c);
CTResult SMStringSetSize(SMString * obj, size_t size);
CTResult SMStringSetCapacity(SMString * obj, size_t cap);
CTResult SMStringAppendFormat(SMString * obj, const char *fmt, ...);
CTResult SMStringSave(SMString * obj, const char *file);

SMSubString SMSubStringWithStr(char *srt);
SMSubString SMSubStringScanUpBeforeFirstCharInStr(SMSubString *str, const char * dlm);
SMSubString SMSubStringScanUpBeforeFirstOccurenceOfStr(SMSubString *str, const char * tok);
SMSubString SMSubStringScanUpBeforeFirstOccurenceOfChar(SMSubString *str, char c);
SMSubString SMSubStringScanUpBeforeFirstUngroupedCharInStr(SMSubString *str, const char * dlm);

SMSubString SMSubStringScanUpAllCharsInStr(SMSubString *str, const char * dlm);

int SMSubStringScanChar(SMSubString *str, char c);

SMSubString SMSubStringScanGroup(SMSubString *str);

void SMSubStringRemoveSurroundingWhiteSpace(SMSubString *str);


#pragma mark - SMArray

typedef struct _SMArray {
    size_t      size;
    size_t      count;
    void     ** data;
} SMArray;

SMArray * SMArrayCreate();
void SMArrayDealloc(SMArray *obj);

// Searches a sorted array
bool SMArraySearch(SMArray *obj, CTUInteger *loc, void *elem, int(*comp)(void*,void*));

CTResult SMArrayInsertElemAtIndex(SMArray *obj, CTUInteger idx, void *elem);
CTResult SMArrayAddElemAtTop(SMArray *obj, void *elem);
CTResult SMArrayAddElem(SMArray *obj, void *elem);
CTResult SMArrayRemoveElemAtIndex(SMArray *obj, CTUInteger idx);


#pragma mark - SMElements

int SMElementCompare(SMElement *elem1, SMElement *elem2);
int SMElementReplace(SMElement **elem, SMElement *oldE, SMElement *newE);
SMElement *SMElementCopy(SMElement *elem);

void SMElementPrint(SMElement *elem);

typedef enum _SMRepresentation {
    
    SMCRepresentation,
    SMMatLabRepresentation
    
} SMRepresentation;

CTResult SMElementToRepresentation(SMElement *elem, SMRepresentation rep, SMString *str);


#pragma mark • SMSymbolElement

typedef struct _SMSymbolElement {
    SMElementType   type;
    SMString      * name;
} SMSymbolElement;

SMSymbolElement * SMSymbolElementCreateWithName(SMString *name);
void SMSymbolElementDealloc(SMSymbolElement *obj);


#pragma mark • SNumberElement

typedef struct _SMNumberElement {
    SMElementType   type;
    double          value;
} SMNumberElement;

SMNumberElement * SMNumberElementCreateWithValue(double val);
void SMNumberElementDealloc(SMNumberElement *obj);


#pragma mark • SMGroupElement

typedef struct _SMGroupElement {
    SMElementType   type;
    SMArray       * elems;
} SMGroupElement;

SMGroupElement * SMGroupElementCreate();
void SMGroupElementDealloc(SMGroupElement *obj);


#pragma mark • SMMatrixElement

typedef struct _SMMatrixElement {
    SMElementType   type;
    SMArray       * elems;
    size_t          rows;
    size_t          cols;
} SMMatrixElement;

SMMatrixElement * SMMatrixElementCreate();
void SMMatrixElementDealloc(SMMatrixElement *obj);


#pragma mark • SMFunElement

typedef struct _SMFunElement {
    SMElementType   type;
    SMArray       * elems;
    SMString      * name;
} SMFunElement;

SMFunElement * SMFunElementCreateWithName(SMString *name);
void SMFunElementDealloc(SMFunElement *obj);


# pragma mark general

void SMGroupElementPrint(SMGroupElement *obj, const char *sep);
void SMSymbolElementPrint(SMSymbolElement *obj);
void SMNumberElementPrint(SMNumberElement *obj);
void SMMultElementPrint(SMGroupElement *obj);
void SMMultElementPrint(SMGroupElement *obj);
void SMAddElementPrint(SMGroupElement *obj);
void SMDivElementPrint(SMGroupElement *obj);
void SMPowElementPrint(SMGroupElement *obj);
void SMFunElementPrint(SMFunElement *obj);
void SMSubMatrixElementPrint(SMFunElement *obj);
void SMMatrixElementPrint(SMMatrixElement *obj);

CTResult SMGroupElementToRepresentation(SMGroupElement *obj, SMRepresentation rep, SMString *str, const char *sep);
CTResult SMMatrixElementToRepresentation(SMMatrixElement *obj, SMRepresentation rep, SMString *str);

CTResult SMElementFactorize(SMElement *el, SMElement **f, SMElement **r, SMSymbolElement *x);
CTResult SMElementSimpleCoeffs(SMElement *el, SMArray *c, SMArray *t, const SMArray *x);
CTResult SMElementForcedSimpleCoeffs(SMElement *el, SMArray *c, SMArray *t, const SMArray *x);

#pragma mark - SMVar

typedef struct _SMVar {
    SMSymbolElement * sym;
    SMElement       * elem;
} SMVar;

SMVar * SMVarCreate();
void SMVarDealloc(SMVar *obj);
CTResult SMVarRepresenatiion(SMVar *var, SMRepresentation rep, SMString *str);


#pragma mark - SMContext

typedef struct _SMContext {
    SMArray * vars;
    SMArray * infoVars;
    SMArray * symNames; // SMString array
} SMContext;

SMContext * SMContextCreate();
void SMContextDealloc(SMContext * obj);

SMSubString SMGetOctaveMatrix(SMSubString subStr);
SMSubString SMGetMaximaMatrix(SMSubString subStr);
SMSubString SMGetMatrix(SMSubString subStr);

SMString * SMContextStringForStr(SMContext * obj, const char *name);
SMString * SMContextStringForSubString(SMContext * obj, SMSubString name);

CTResult SMContextGetElementsOfType(SMContext * obj, SMElementType type, SMArray *elems);
CTResult SMContextGetUknownSymbols(SMContext * obj, SMArray *elems);

CTResult SMContextAddVar(SMContext * obj, SMVar *var);

CTResult SMContextRepresenation(SMContext * obj, SMRepresentation rep, SMString *str);

CTResult SMContextLoadFromString(SMContext * obj, SMString *str);
CTResult SMContextLoadFromSubString(SMContext * obj, SMSubString str);
CTResult SMContextLoadInfoFromSubString(SMContext * obj, SMSubString str);
CTResult SMContextLoadInfoFromString(SMContext * obj, SMString *str);
SMElement * SMContextScanMatrix(SMContext * obj, SMSubString data);



#endif /* Symbolic_h */
