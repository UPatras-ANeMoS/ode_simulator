//
//  Symbolic.c
//  SymToODE
//
//  Created by Kostis Giannousakis on 26/01/16.
//  Copyright © 2016 Kostis Giannousakis. All rights reserved.
//

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <ctype.h>
#include "Symbolic.h"


void SMElementDealloc(SMElement *obj) {
    if (obj) {
        switch (obj->type & ~SMElementTypeNegativeMask) {
            case SMElementTypeSymbol:
                SMSymbolElementDealloc((SMSymbolElement *)obj);
                break;
            case SMElementTypeNumber:
                SMNumberElementDealloc((SMNumberElement *)obj);
                break;
            case SMElementTypeMult:
            case SMElementTypeAdd:
            case SMElementTypeDiv:
            case SMElementTypePow:
            case SMElementTypeGroup:
                SMGroupElementDealloc((SMGroupElement *)obj);
                break;
            case SMElementTypeMatrix:
                SMMatrixElementDealloc((SMMatrixElement *)obj);
                break;
            case SMElementTypeFun:
            case SMElementTypeSubMatrix:
                SMFunElementDealloc((SMFunElement *)obj);
                break;
        }
    }
}

void SMElementPrint(SMElement *obj) {
    if (obj) {
#define SMPrintFlat
#ifdef SMPrintFlat
        switch (obj->type & ~SMElementTypeNegativeMask) {
            case SMElementTypeSymbol:
                SMSymbolElementPrint((SMSymbolElement *)obj);
                break;
            case SMElementTypeNumber:
                SMNumberElementPrint((SMNumberElement *)obj);
                break;
            case SMElementTypeMult:
                SMMultElementPrint((SMGroupElement *)obj);
                break;
            case SMElementTypeAdd:
                SMAddElementPrint((SMGroupElement *)obj);
                break;
            case SMElementTypeDiv:
                SMDivElementPrint((SMGroupElement *)obj);
                break;
            case SMElementTypePow:
                SMPowElementPrint((SMGroupElement *)obj);
                break;
            case SMElementTypeMatrix:
                SMMatrixElementPrint((SMMatrixElement *)obj);
                break;
            case SMElementTypeFun:
                SMFunElementPrint((SMFunElement *)obj);
            case SMElementTypeSubMatrix:
                SMSubMatrixElementPrint((SMFunElement *)obj);
                break;
        }
#else
        switch (obj->type & ~SMElementTypeNegativeMask) {
            case SMElementTypeSymbol:
                printf("Sym("), SMSymbolElementPrint((SMSymbolElement *)obj), printf(")");
                break;
            case SMElementTypeNumber:
                printf("Num("), SMNumberElementPrint((SMNumberElement *)obj), printf(")");
                break;
            case SMElementTypeMult:
                printf("Mult("), SMMultElementPrint((SMGroupElement *)obj), printf(")");
                break;
            case SMElementTypeAdd:
                printf("Add("), SMAddElementPrint((SMGroupElement *)obj), printf(")");
                break;
            case SMElementTypeDiv:
                printf("Div("), SMDivElementPrint((SMGroupElement *)obj), printf(")");
                break;
            case SMElementTypePow:
                printf("Pow("), SMPowElementPrint((SMGroupElement *)obj), printf(")");
                break;
            case SMElementTypeMatrix:
                SMMatrixElementPrint((SMMatrixElement *)obj);
                break;
            case SMElementTypeFun:
                SMFunElementPrint((SMFunElement *)obj);
            case SMElementTypeSubMatrix:
                SMSubMatrixElementPrint((SMFunElement *)obj);
                break;
        }
#endif
    }
}

void SMSymbolElementPrint(SMSymbolElement *obj)
{
    if (obj->type & SMElementTypeNegativeMask)
        printf("(-");
    
    printf("%s",obj->name->data);
    
    if (obj->type & SMElementTypeNegativeMask)
        printf(")");
}

void SMNumberElementPrint(SMNumberElement *obj)
{
    if (obj->type & SMElementTypeNegativeMask)
        printf("(-");
    
    printf("%g", obj->value);
    
    if (obj->type & SMElementTypeNegativeMask)
        printf(")");
}

void SMMultElementPrint(SMGroupElement *obj) {
    SMGroupElementPrint(obj, "*");
}

void SMAddElementPrint(SMGroupElement *obj) {
    SMGroupElementPrint(obj, " + ");
}

void SMDivElementPrint(SMGroupElement *obj) {
    SMGroupElementPrint(obj, "/");
}

void SMPowElementPrint(SMGroupElement *obj) {
    SMGroupElementPrint(obj, "^");
}

void SMGroupElementPrint(SMGroupElement *obj, const char *sep) {
    
#ifndef SMPrintFlat
    sep = ", ";
#endif
    
    if (obj->type & SMElementTypeNegativeMask)
        printf("(-");
    
    for (size_t i = 0; i < obj->elems->count; i++, printf("%s", i < obj->elems->count ? sep:"")) {
        SMElement *elem = obj->elems->data[i];
        SMElementType currType = obj->type & ~SMElementTypeNegativeMask;
        SMElementType newType  = elem->type & ~SMElementTypeNegativeMask;
        if (newType > currType && newType <= SMElementTypeAdd) {
            printf("(");
            SMElementPrint(elem);
            printf(")");
        }
        else
            SMElementPrint(elem);
    }
    
    if (obj->type & SMElementTypeNegativeMask)
        printf(")");
}

void SMFunElementPrint(SMFunElement *obj) {
    
    if (obj->type & SMElementTypeNegativeMask)
            printf("(-");
    
    printf("%s(", obj->name->data);
    
    SMElementPrint(obj->elems->data[0]);
    for (size_t i = 1; i < obj->elems->count; i++) {
        printf(", ");
        SMElementPrint(obj->elems->data[i]);
    }
    
    printf(")");
    
    if (obj->type & SMElementTypeNegativeMask)
        printf(")");
}

void SMSubMatrixElementPrint(SMFunElement *obj) {
    
    if (obj->type & SMElementTypeNegativeMask)
        printf("(-");
    
    printf("%s[", obj->name->data);
    
    SMElementPrint(obj->elems->data[0]);
    for (size_t i = 1; i < obj->elems->count; i++) {
        printf(", ");
        SMElementPrint(obj->elems->data[i]);
    }
    
    printf("]");
    
    if (obj->type & SMElementTypeNegativeMask)
        printf(")");
}

void SMMatrixElementPrint(SMMatrixElement *obj)
{
    for (size_t i = 0; i < obj->rows; i++, printf("%s", i < obj->rows ? "\n":"")) {
        for (size_t j = 0; j < obj->cols; j++, printf("%s", j < obj->cols ? ", ":"")) {
            SMElementPrint(obj->elems->data[i*obj->cols + j]);
        }
    }
}

int SMSymbolElementCompare(SMSymbolElement *elem1, SMSymbolElement *elem2) {
    
//    if (strcmp(elem1->name->data, "wxd")==0)
//    {
//        printf("%s == %s: %d\n", elem1->name->data, elem2->name->data, strcmp(elem1->name->data, elem2->name->data));
//        
//        
//    }
    
    return strcmp(elem1->name->data, elem2->name->data);
}

int SMNumberElementCompare(SMNumberElement *elem1, SMNumberElement *elem2) {
    if (elem1->value > elem2->value)
        return 1;
    else if (elem1->value < elem2->value)
        return -1;
    
    return 0;
}

int SMGroupElementCompare(SMGroupElement *elem1, SMGroupElement *elem2) {
    
    if (elem1->elems->count > (elem2->elems->count))
        return 1;
    else if (elem1->elems->count < (elem2->elems->count))
        return -1;
    
    int res = 0;
    for (size_t i = 0; i < elem1->elems->count && !res; i++) {
        res = SMElementCompare(elem1->elems->data[i], elem2->elems->data[i]);
    }
    
    return res;
}

int SMFunElementCompare(SMFunElement *elem1, SMFunElement *elem2)
{
    int res = strcmp(elem1->name->data, elem2->name->data);
    
    for (size_t i = 0; i < elem1->elems->count && !res; i++) {
        res = SMElementCompare(elem1->elems->data[i], elem2->elems->data[i]);
    }
    
    return res;
}

CTUInteger SMElementArrayIndexOfElement(SMArray *elems, SMElement *el)
{
    for (CTUInteger i = 0; i < elems->count; i++)
        if (!SMElementCompare(elems->data[i], el))
            return i;
    
    return CTNotFound;
}

CTResult SMElementGetElementsOfType(SMElement *el, SMElementType type, SMArray *elems)
{
    CTResult res;
    
    if ((el->type & ~SMElementTypeNegativeMask) == type)
    {
        SMElementType rType = el->type;
        el->type &= ~SMElementTypeNegativeMask;
        CTUInteger idx = SMElementArrayIndexOfElement(elems, el);
        el->type = rType;
        
        if (idx == CTNotFound) {
            SMElement *copy = SMElementCopy(el);
            copy->type &= ~SMElementTypeNegativeMask;
            if (CTSuccess != (res = SMArrayAddElem(elems, copy))) {
                SMElementDealloc(copy);
                return res;
            }
        }
    }
    
    if ((el->type & ~SMElementTypeNegativeMask) >= SMElementTypeGroup) {
        SMGroupElement * gel = (SMGroupElement *)el;
        for (CTUInteger i = 0; i < gel->elems->count; i++) {
            if (CTSuccess != (res = SMElementGetElementsOfType(gel->elems->data[i], type, elems)))
                return res;
        }
    }
    
    return CTSuccess;
}

// TODO: mind '-'
int SMElementCompare(SMElement *elem1, SMElement *elem2)
{
    if (elem1->type > elem2->type)
        return 1;
    else if (elem1->type < elem2->type)
        return -1;
    else
    {
        switch (elem1->type & ~SMElementTypeNegativeMask) {
            case SMElementTypeSymbol:
                return SMSymbolElementCompare((SMSymbolElement *)elem1, (SMSymbolElement *)elem2);
            case SMElementTypeNumber:
                return SMNumberElementCompare((SMNumberElement *)elem1, (SMNumberElement *)elem2);
            case SMElementTypeMult:
            case SMElementTypeAdd:
            case SMElementTypeDiv:
            case SMElementTypePow:
            case SMElementTypeGroup:
                return SMGroupElementCompare((SMGroupElement *)elem1, (SMGroupElement *)elem2);
            case SMElementTypeFun:
            case SMElementTypeSubMatrix:
                return SMFunElementCompare((SMFunElement *)elem1, (SMFunElement *)elem2);
            default:
                return 1; // error
        }
    }
}

SMElement *SMElementCopy(SMElement *elem)
{
    SMElement *newEl = NULL;
    SMElementType type = elem->type & ~SMElementTypeNegativeMask;
    if (type == SMElementTypeNumber) {
        SMNumberElement *old = (SMNumberElement *)elem;
        SMNumberElement *new = SMNumberElementCreateWithValue(old->value);
        new->type = old->type;
        newEl = (SMElement *)new;
    }
    else if (type == SMElementTypeSymbol) {
        SMSymbolElement *old = (SMSymbolElement *)elem;
//        SMSymbolElement *new = SMSymbolElementCreateWithName(SMStringCreateWithString(old->name->data));
        SMSymbolElement *new = SMSymbolElementCreateWithName((old->name));
        new->type = old->type;
        newEl = (SMElement *)new;
    }
    else if (type == SMElementTypeFun || type == SMElementTypeSubMatrix) {
        SMFunElement *old = (SMFunElement *)elem;
//        SMFunElement *new = SMFunElementCreateWithName(SMStringCreateWithString(old->name->data));
        SMFunElement *new = SMFunElementCreateWithName((old->name));
        new->type = old->type;
        newEl = (SMElement *)new;
    }
    else if (type == SMElementTypeMatrix) {
        SMMatrixElement *old = (SMMatrixElement *)elem;
        SMMatrixElement *new = SMMatrixElementCreate();
        new->type = old->type;
        new->cols = old->cols;
        new->rows = old->rows;
        newEl = (SMElement *)new;
    }
    else if (type > SMElementTypeGroup)
    {
        SMGroupElement *old = (SMGroupElement *)elem;
        SMGroupElement *new = SMGroupElementCreate();
        new->type = old->type;
        newEl = (SMElement *)new;
    }
    
    if (type > SMElementTypeGroup) {
        SMGroupElement *old = (SMGroupElement *)elem;
        SMGroupElement *new = (SMGroupElement *)newEl;
        
        for (size_t i = 0; i < old->elems->count; i++) {
            SMArrayAddElem(new->elems, SMElementCopy(old->elems->data[i]));
        }
    }
    
    return newEl;
}

// FIXME: No memory management
int SMElementReplace(SMElement **elem, SMElement *oldE, SMElement *newE)
{
    int count = 0;
    if (!SMElementCompare(*elem, oldE))
    {
        SMElementType mask = (*elem)->type & SMElementTypeNegativeMask;
        SMElementDealloc(*elem);
        *elem = SMElementCopy(newE);
        
        if (mask) {
            if ((*elem)->type & SMElementTypeNegativeMask) // Remove
                (*elem)->type &= ~SMElementTypeNegativeMask;
            else    // Add
                (*elem)->type |= SMElementTypeNegativeMask;
        }
        
        count++;
    }
    else if (((*elem)->type & ~SMElementTypeNegativeMask) >= SMElementTypeGroup)
    {
        SMGroupElement *group = (SMGroupElement *)*elem;
        for (size_t i = 0; i < group->elems->count; i++)
            count += SMElementReplace((SMElement **)&group->elems->data[i], oldE, newE);
    }
    
    return count;
}


CTResult SMSymbolElementToRepresentation(SMSymbolElement *obj, SMRepresentation rep, SMString *str)
{
    if (obj->type & SMElementTypeNegativeMask)
        return SMStringAppendFormat(str, "(-%s)", obj->name->data);
    else
        return SMStringAppendFormat(str, obj->name->data);
}

CTResult SMNumberElementToRepresentation(SMNumberElement *obj, SMRepresentation rep, SMString *str)
{
    CTResult res;
    
    char buf[100];
    
    sprintf(buf, "%g", obj->value);
    if (rep == SMCRepresentation && !strchr(buf, '.') && !strchr(buf, 'e') && !strchr(buf, 'E'))
        strncat(buf, ".", 1);
    
    if (obj->type & SMElementTypeNegativeMask)
        res = SMStringAppendFormat(str, "(-%s)", buf);
    else
        res = SMStringAppendFormat(str, "%s", buf);
    
    return res;
}

CTResult SMGroupElementToRepresentation(SMGroupElement *obj, SMRepresentation rep, SMString *str, const char *sep)
{
    CTResult res = CTSuccess;
    if (obj->type & SMElementTypeNegativeMask &&
        CTSuccess != (res = SMStringAppendFormat(str, "(-")))
        return res;
    
    for (size_t i = 0; i < obj->elems->count && res == CTSuccess; i++, res = SMStringAppendFormat(str, "%s", i < obj->elems->count ? sep:"")) {
        SMElement *elem = obj->elems->data[i];
        SMElementType currType = obj->type & ~SMElementTypeNegativeMask;
        SMElementType newType  = elem->type & ~SMElementTypeNegativeMask;
        if (newType > currType && newType <= SMElementTypeAdd) {
            if (CTSuccess != (res = SMStringAppendFormat(str, "(")) ||
                CTSuccess != (res = SMElementToRepresentation(elem, rep, str)) ||
                CTSuccess != (res = SMStringAppendFormat(str, ")")))
                return res;
        }
        else
            if (CTSuccess != (res = SMElementToRepresentation(elem, rep, str)))
                return res;
    }
    
    if (obj->type & SMElementTypeNegativeMask &&
        CTSuccess != (res = SMStringAppendFormat(str, ")")))
        return res;
    
    return CTSuccess;
}

CTResult SMMultElementToRepresentation(SMGroupElement *obj, SMRepresentation rep, SMString *str) {
    return SMGroupElementToRepresentation(obj, rep, str, "*");
}

CTResult SMAddElementToRepresentation(SMGroupElement *obj, SMRepresentation rep, SMString *str)
{
    CTResult res = CTSuccess;
    if (obj->type & SMElementTypeNegativeMask &&
        CTSuccess != (res = SMStringAppendFormat(str, "(-(")))
        return res;
    
    for (size_t i = 0; i < obj->elems->count && res == CTSuccess; i++) {
        SMElement *elem = obj->elems->data[i];
        
        if (elem->type & SMElementTypeNegativeMask)
        {
            if (i > 0)
                if (CTSuccess != (res = SMStringAppendFormat(str, " - ")))
                    return res;
            
            elem->type &= ~SMElementTypeNegativeMask;
            
            if (CTSuccess != (res = SMElementToRepresentation(elem, rep, str)))
                return res;
            
            elem->type |= SMElementTypeNegativeMask;
        }
        else
        {
            if (i > 0)
                if (CTSuccess != (res = SMStringAppendFormat(str, " + ")))
                    return res;
            
            if (CTSuccess != (res = SMElementToRepresentation(elem, rep, str)))
                return res;
        }
    }
    
    if (obj->type & SMElementTypeNegativeMask &&
        CTSuccess != (res = SMStringAppendFormat(str, "))")))
        return res;
    
    return CTSuccess;
}

CTResult SMDivElementToRepresentation(SMGroupElement *obj, SMRepresentation rep, SMString *str) {
    return SMGroupElementToRepresentation(obj, rep, str, "/");
}

CTResult SMPowElementToRepresentation(SMGroupElement *obj, SMRepresentation rep, SMString *str)
{
    if (rep == SMCRepresentation) {
        CTResult res;
        
        SMFunElement   *pow  = SMFunElementCreateWithName(SMStringCreateWithString("pow"));
//        SMFunElement   *pow  = SMFunElementCreateWithName(SMContextStringForStr(obj, "pow"));
        SMGroupElement *mult = SMGroupElementCreate();
        
        mult->type = SMElementTypeMult;
        if (obj->type & SMElementTypeNegativeMask)
            pow->type |= SMElementTypeNegativeMask;
        
        SMArrayAddElem(pow->elems, obj->elems->data[0]);
        SMArrayAddElem(pow->elems, mult);
        
        for (int i = 1; i < obj->elems->count; i++) {
            SMArrayAddElem(mult->elems, obj->elems->data[i]);
        }
        
        res = SMElementToRepresentation((SMElement *)pow, rep, str);
        
        // Remove elems as we don't own them
        pow->elems->count = 0;
        mult->elems->count = 0;
        SMGroupElementDealloc(mult);
        SMStringDealloc(pow->name);
        SMFunElementDealloc(pow);
        
        return res;
    }
    else
        return SMGroupElementToRepresentation(obj, rep, str, "^");
}



CTResult SMFunElementToRepresentation(SMFunElement *obj, SMRepresentation rep, SMString *str)
{
    CTResult res = CTSuccess;
    if (obj->type & SMElementTypeNegativeMask &&
        CTSuccess != (res = SMStringAppendFormat(str, "(-")))
        return res;
    
    if (CTSuccess != (res = SMStringAppendFormat(str, "%s(", obj->name->data)))
        return res;
    
    const char *sep = ", ";
    
    for (size_t i = 0; i < obj->elems->count && res == CTSuccess; i++, res = SMStringAppendFormat(str, "%s", i < obj->elems->count ? sep:"")) {
        SMElement *elem = obj->elems->data[i];
        SMElementType currType = obj->type & ~SMElementTypeNegativeMask;
        SMElementType newType  = elem->type & ~SMElementTypeNegativeMask;
        if (newType > currType && newType <= SMElementTypeAdd) {
            if (CTSuccess != (res = SMStringAppendFormat(str, "(")) ||
                CTSuccess != (res = SMElementToRepresentation(elem, rep, str)) ||
                CTSuccess != (res = SMStringAppendFormat(str, ")")))
                return res;
        }
        else
            if (CTSuccess != (res = SMElementToRepresentation(elem, rep, str)))
                return res;
    }
    
    if (CTSuccess != (res = SMStringAppendFormat(str, ")")))
        return res;
    
    if (obj->type & SMElementTypeNegativeMask &&
        CTSuccess != (res = SMStringAppendFormat(str, ")")))
        return res;
    
    return CTSuccess;
}

CTResult SMSubMatrixElementToRepresentation(SMFunElement *obj, SMRepresentation rep, SMString *str)
{
    CTResult res = CTSuccess;
    if (obj->type & SMElementTypeNegativeMask &&
        CTSuccess != (res = SMStringAppendFormat(str, "(-")))
        return res;
    
    if (CTSuccess != (res = SMStringAppendFormat(str, "%s", obj->name->data)))
        return res;
    
    for (size_t i = 0; i < obj->elems->count && res == CTSuccess; i++) {
        SMElement *elem = obj->elems->data[i];
        if ((elem->type & ~SMElementTypeNegativeMask) == SMElementTypeNumber){
            if (CTSuccess != (res = SMStringAppendFormat(str, "[%s%d]", elem->type & SMElementTypeNegativeMask ? "-":"", (int)((SMNumberElement *)elem)->value)))
                return res;
        }
        else if (CTSuccess != (res = SMStringAppendFormat(str, "[")) ||
                CTSuccess != (res = SMElementToRepresentation(elem, rep, str)) ||
                CTSuccess != (res = SMStringAppendFormat(str, "]")))
                return res;
    }
    
    if (obj->type & SMElementTypeNegativeMask &&
        CTSuccess != (res = SMStringAppendFormat(str, ")")))
        return res;
    
    return CTSuccess;
}


CTResult SMMatrixElementToRepresentation(SMMatrixElement *obj, SMRepresentation rep, SMString *str)
{
    CTResult res = CTSuccess;
    
    char g_op;
    char g_cl;
    
    if (rep == SMCRepresentation) {
        g_op = '{';
        g_cl = '}';
        
    } else {
        g_op = '[';
        g_cl = ']';
    }
        
    
    if (CTSuccess != (res = SMStringAppendFormat(str, "%c\n", g_op)))
        return res;
    
    for (size_t i = 0; i < obj->rows && res == CTSuccess; i++, res = SMStringAppendFormat(str, "%s", i < obj->rows ? ",\n":"")) {
        if (CTSuccess != (res = SMStringAppendFormat(str, "    %c", g_op)))
            return res;
        for (size_t j = 0; j < obj->cols && res == CTSuccess; j++, res = SMStringAppendFormat(str, "%s", j < obj->cols ? ", ":"")) {
            if (CTSuccess != (res = SMElementToRepresentation(obj->elems->data[i*obj->cols + j], rep, str)))
                return res;
        }
        if (CTSuccess != (res = SMStringAppendFormat(str, "%c", g_cl)))
            return res;
    }
    
    if (CTSuccess != (res = SMStringAppendFormat(str, "\n%c", g_cl)))
        return res;
    
    return CTSuccess;
}

CTResult SMElementToRepresentation(SMElement *obj, SMRepresentation rep, SMString *str)
{
    switch (obj->type & ~SMElementTypeNegativeMask) {
        case SMElementTypeSymbol:
            return SMSymbolElementToRepresentation((SMSymbolElement *)obj, rep, str);
        case SMElementTypeNumber:
            return SMNumberElementToRepresentation((SMNumberElement *)obj, rep, str);
        case SMElementTypeMult:
            return SMMultElementToRepresentation((SMGroupElement *)obj, rep, str);
        case SMElementTypeAdd:
            return SMAddElementToRepresentation((SMGroupElement *)obj, rep, str);
        case SMElementTypeDiv:
            return SMDivElementToRepresentation((SMGroupElement *)obj, rep, str);
        case SMElementTypePow:
            return SMPowElementToRepresentation((SMGroupElement *)obj, rep, str);
        case SMElementTypeMatrix:
            return SMMatrixElementToRepresentation((SMMatrixElement *)obj, rep, str);
            break;
        case SMElementTypeFun:
            return SMFunElementToRepresentation((SMFunElement *)obj, rep, str);
        case SMElementTypeSubMatrix:
            return SMSubMatrixElementToRepresentation((SMFunElement *)obj, rep, str);
    }
    
    return CTError;
}

#pragma mark - SMString

#define SMStringAllocationStep 100

SMString * SMStringCreate() {
    SMString *obj = malloc(sizeof(SMString));
    if (obj) {
        obj->len  = 0;
        obj->size = 0;
        obj->data = NULL;
    }
    
    return obj;
}

CTResult SMStringEmpty(SMString * obj)
{
    CTResult res;
    obj->len = 0;
    if (CTSuccess != (res = SMStringSetCapacity(obj, 0)))
        return res;
    
    *obj->data = 0;
    
    return CTSuccess;
}

CTResult SMStringSetSize(SMString * obj, size_t size)
{
    char *data = realloc(obj->data, size * sizeof(char));
    if (!data) return errno;
    
    obj->data = data;
    obj->size = size;
    
    if (!size)
        obj->len = 0;
    else if (obj->len+1 > size) {
        obj->data[size-1] = 0;
        obj->len = size-1;
    }
    
    return CTSuccess;
}

CTResult SMStringSetCapacity(SMString * obj, size_t cap) {
    return (obj->len + cap +1 > obj->size) ? SMStringSetSize(obj, obj->len + cap +1):CTSuccess;
}

CTResult SMStringSave(SMString * obj, const char *file)
{
    FILE *f = fopen(file, "w");
    if (!f) return CTError;
    fprintf(f, "%s", obj->data);
    fclose(f);
    
    return CTSuccess;
}

CTResult SMStringAppendFormat(SMString * obj, const char *fmt, ...)
{
    va_list valist;
    va_start(valist, fmt);
    int len = vsnprintf(NULL, 0, fmt, valist);
    va_end(valist);
    
    CTResult res;
    if (CTSuccess != (res = SMStringSetCapacity(obj, len)))
        return res;
    
    va_start(valist, fmt);
    vsprintf(obj->data+obj->len, fmt, valist);
    va_end(valist);
    
    obj->len += len;
    
    return CTSuccess;
}

CTResult SMStringAppendChar(SMString * obj, char c) {
    CTResult res;
    if (obj->len +1+1 > obj->size &&
        CTSuccess != (res = SMStringSetSize(obj, obj->size + SMStringAllocationStep)))
        return res;
    
    obj->data[obj->len++] = c;
    obj->data[obj->len]   = 0;
    
    return CTSuccess;
}

CTResult SMStringLoadContentsOfFile(SMString * obj, const char *file)
{
    FILE *f = fopen(file, "r");
    if (!f) return errno;
    
    CTResult res = CTSuccess;
    int c;
    while ((c = fgetc(f)) > 0 && CTSuccess == (res = SMStringAppendChar(obj, c)));
    
    fclose(f);
    
    return res;
}

SMString * SMStringCreateWithString(const char *str) {
    SMString *obj = SMStringCreate();
    if (obj) {
        obj->len  = strlen(str);
        obj->size = 1+obj->len;
        
        if (!(obj->data = malloc(obj->size * sizeof(char)))) {
            SMStringDealloc(obj);
            return NULL;
        }
        
        strcpy(obj->data, str);
    }
    
    return obj;
}

SMString * SMStringCreateWithSubString(SMSubString str)
{
    SMString *obj = SMStringCreate();
    if (obj) {
        obj->len  = str.len;
        obj->size = 1+obj->len;
        
        if (!(obj->data = malloc(obj->size * sizeof(char)))) {
            SMStringDealloc(obj);
            return NULL;
        }
        
        strncpy(obj->data, str.data, str.len);
        obj->data[obj->len] = 0;
    }
    
    return obj;
}

void SMStringDealloc(SMString *obj) {
    if (obj) {
        free(obj->data);
        
        free(obj);
    }
}

SMSubString SMSubStringWithStr(char *srt)
{
    return (SMSubString){srt, strlen(srt)};
}

SMSubString SMSubStringScanUpBeforeFirstOccurenceOfStr(SMSubString *str, const char * tok)
{
    SMSubString subStr;
    CTUInteger len = strlen(tok);
    for (subStr.data = str->data, subStr.len = (SMSubStringScanUpBeforeFirstOccurenceOfChar(str, *tok)).len;
         str->len && strncmp(str->data, tok, len);
         SMSubStringScanChar(str, *tok),
         subStr.len += (SMSubStringScanUpBeforeFirstOccurenceOfChar(str, *tok)).len);
    
    return subStr;
}

SMSubString SMSubStringScanUpBeforeFirstCharInStr(SMSubString *str, const char * dlm)
{
    SMSubString subStr = *str;
    for (subStr.len = 0; str->len && !strchr(dlm, *str->data); subStr.len++, str->data++, str->len--);
    
    return subStr;
}

SMSubString SMSubStringScanUpBeforeFirstOccurenceOfChar(SMSubString *str, char c)
{
    SMSubString subStr = *str;
    for (subStr.len = 0; str->len && c != *str->data; subStr.len++, str->data++, str->len--);
    
    return subStr;
}

SMSubString SMSubStringScanUpAllCharsInStr(SMSubString *str, const char * dlm)
{
    SMSubString subStr = *str;
    for (subStr.len = 0; str->len && strchr(dlm, *str->data); subStr.len++, str->data++, str->len--);
    
    return subStr;
}

SMSubString SMSubStringScanUpAllTrailingCharsInStr(SMSubString *str, const char * dlm)
{
    SMSubString subStr;
    for (subStr.data = &str->data[str->len], subStr.len = 0; str->len && strchr(dlm, str->data[str->len-1]); subStr.len++, subStr.data--, str->len--);
    
    return subStr;
}

void SMSubStringRemoveSurroundingWhiteSpace(SMSubString *str)
{
    const char *ws = " \t\f\r\n\v";
    SMSubStringScanUpAllCharsInStr(str, ws);
    SMSubStringScanUpAllTrailingCharsInStr(str, ws);
}

int SMSubStringScanChar(SMSubString *str, char c)
{
    if (str->len && str->data[0] == c)
    {
        str->data++;
        str->len--;
        return 1;
    }
    
    return 0;
}

// TODO: Error handling
SMSubString SMSubStringScanUpBeforeFirstUngroupedCharInStr(SMSubString *str, const char * dlm)
{
    SMString *strDel = SMStringCreateWithString("([{<");
    SMStringAppendFormat(strDel, dlm);
    
    SMSubString subStr = *str;

    for (SMSubStringScanUpBeforeFirstCharInStr(str, strDel->data);
         str->len && !strchr(dlm, *str->data);
         SMSubStringScanGroup(str),
         SMSubStringScanUpBeforeFirstCharInStr(str, strDel->data));
    
    subStr.len = str->data - subStr.data;
    
    SMStringDealloc(strDel);
    
    return subStr;
}

// FIXME: Stack if group cannot be closed
SMSubString SMSubStringScanGroup(SMSubString *str)
{
    SMSubString subStr = {str->data, 0};
    SMString *groups = SMStringCreate();
    
    if (str->len && strchr("([{<", *str->data)) {
        SMStringAppendChar(groups, *str->data);
        str->data++;
        str->len--;
        subStr.data++;
    }
    else return subStr;
    
    while (groups->len)
    {
        SMSubString tmp = SMSubStringScanUpBeforeFirstCharInStr(str, "([{<)]}>");
        char c = *str->data;
        if (strchr("([{<", c)) {
            SMStringAppendChar(groups, c);
        }
        else if (c == ')')
        {
            if (groups->data[groups->len-1] == '(')
                groups->len--;
        }
        else if (c == ']')
        {
            if (groups->data[groups->len-1] == '[')
                groups->len--;
        }
        else if (c == '}')
        {
            if (groups->data[groups->len-1] == '{')
                groups->len--;
        }
        else if (c == '>')
        {
            if (groups->data[groups->len-1] == '<')
                groups->len--;
        }
        else break; // Error
        
        subStr.len += tmp.len+1;
        str->data++;
        str->len--;
    }
    
    subStr.len--;
    
    SMStringDealloc(groups);
    
    return subStr;
}


#pragma mark - SMArray

SMArray * SMArrayCreate() {
    SMArray * obj = malloc(sizeof(SMArray));
    if (obj) {
        obj->size = obj->count = 0;
        obj->data = NULL;
    }
    
    return obj;
}

void SMArrayDealloc(SMArray *obj) {
    if (obj) {
        free(obj->data);
        
        free(obj);
    }
}

bool SMArraySearch(SMArray *obj, CTUInteger *loc, void *elem, int(*comp)(void*,void*))
{
    CTUInteger i;
    int res = 1;
    for (i = 0; i < obj->count && (res = comp(obj->data[i],elem)) < 0; i++);
    
    *loc = i;

    return !res;
}

# define SMArrayAllocationStep 10

CTResult SMArraySetSize(SMArray *obj, size_t size)
{
    void *data = realloc(obj->data, size * sizeof(void *));
    if (!data) return errno;
    
    obj->data = data;
    obj->size = size;
    
    if (obj->count > size) {
//        obj->data[size-1] = 0;
        obj->count = size;
    }
    
    return CTSuccess;
}

CTResult SMArrayAddElem(SMArray *obj, void *elem) {
    CTResult res;
    if (obj->count +1 > obj->size &&
        CTSuccess != (res = SMArraySetSize(obj, obj->size + SMArrayAllocationStep)))
        return res;
    
    obj->data[obj->count++] = elem;
    
    return CTSuccess;
}

CTResult SMArrayAddElemAtTop(SMArray *obj, void *elem)
{
    CTResult res;
    if (obj->count +1 > obj->size &&
        CTSuccess != (res = SMArraySetSize(obj, obj->size + SMArrayAllocationStep)))
        return res;
    
    for (CTInteger i = obj->count; i > 0; i--) {
        obj->data[i] = obj->data[i-1];
    }
    
    obj->count++;
    obj->data[0] = elem;
    
    return CTSuccess;
}

CTResult SMArrayInsertElemAtIndex(SMArray *obj, CTUInteger idx, void *elem)
{
    if (idx == CTNotFound) return CTError;
    
    CTResult res;
    if (obj->count +1 > obj->size &&
        CTSuccess != (res = SMArraySetSize(obj, obj->size + SMArrayAllocationStep)))
        return res;
    
    for (CTInteger i = obj->count; i > idx; i--) {
        obj->data[i] = obj->data[i-1];
    }
    
    obj->count++;
    obj->data[idx] = elem;
    
    return CTSuccess;
}

CTResult SMArrayRemoveElemAtIndex(SMArray *obj, CTUInteger idx) {
    if (idx >= obj->count)
        return CTError;
    
    for (obj->count--; idx < obj->count; idx++) {
        obj->data[idx] = obj->data[idx+1];
    }
    
    return CTSuccess;
}


#pragma mark - SMElements

CTResult SMElementFactorize(SMElement *el, SMElement **f, SMElement **r, SMSymbolElement *x);

CTResult SMSymbolElementFactorize(SMSymbolElement *el, SMElement **f, SMElement **r, SMSymbolElement *x)
{
    // FIXME: negative ????
    if (0 == SMElementCompare((SMElement *)el, (SMElement *)x))
    {
        *f = (SMElement *)SMNumberElementCreateWithValue(1);
        *r = NULL;
    }
    else
    {
        *f = NULL;
        *r = SMElementCopy((SMElement *)el);
    }
    
    return CTSuccess;
}

CTResult SMMultElementFactorize(SMGroupElement *el, SMElement **f, SMElement **r, SMSymbolElement *x)
{
    SMGroupElement * mEl = (SMGroupElement *)SMElementCopy((SMElement *)el);
//    mEl->type = el->type;
    
    int found = 0;
    CTUInteger i;
    SMElement *fn = NULL, *rn = NULL;
    for (i = 0; i < el->elems->count; i++)
    {
        SMElementFactorize(el->elems->data[i], &fn, &rn, x);
        
        if (fn)
        {
//            if (found) {
//                // Error non-linear
//                SMElementDealloc((SMElement *)mEl);
//                SMElementDealloc(fn);
//                SMElementDealloc(rn);
//                return CTError;
//            }
            
            found = 1;
            
            break;
        }
        else
        {
            SMElementDealloc(rn);
            rn = NULL;
        }
    }
    
    if (i < el->elems->count)
    {
        SMElementDealloc(mEl->elems->data[i]);
        SMArrayRemoveElemAtIndex(mEl->elems, i);
        
        *f = (SMElement *)mEl;
        *r = rn ? SMElementCopy((SMElement *)mEl):NULL;
        
        if (fn->type & SMElementTypeNegativeMask) {
            mEl->type ^= SMElementTypeNegativeMask;
            fn->type  ^= SMElementTypeNegativeMask;
        }
        
        if (fn->type != SMElementTypeNumber || ((SMNumberElement *)fn)->value != 1)
            SMArrayAddElem(mEl->elems, fn);
        else
            SMElementDealloc(fn);
        
        if (mEl->elems->count == 1)
        {
            *f = mEl->elems->data[0];
            if (mEl->type & SMElementTypeNegativeMask)
                (*f)->type ^= SMElementTypeNegativeMask;
            SMArrayRemoveElemAtIndex(mEl->elems, 0);
            SMElementDealloc((SMElement *)mEl);
        }
        
        mEl = (SMGroupElement *)*r;
        if (mEl)
        {
            if (rn->type & SMElementTypeNegativeMask) {
                mEl->type ^= SMElementTypeNegativeMask;
                rn->type  ^= SMElementTypeNegativeMask;
            }
            
            if (rn->type != SMElementTypeNumber || ((SMNumberElement *)rn)->value != 1)
                SMArrayAddElem(mEl->elems, rn);
            else
                SMElementDealloc(rn);
            
            if (mEl->elems->count == 1)
            {
                *r = mEl->elems->data[0];
                if (mEl->type & SMElementTypeNegativeMask)
                    (*r)->type ^= SMElementTypeNegativeMask;
                SMArrayRemoveElemAtIndex(mEl->elems, 0);
                SMElementDealloc((SMElement *)mEl);
            }
        }
    }
    else
    {
        *f = NULL;
        *r = (SMElement *)mEl;
    }
    
    return CTSuccess;
}

CTResult SMAddElementFactorize(SMGroupElement *el, SMElement **_f, SMElement **_r, SMSymbolElement *x)
{
    SMGroupElement *f = SMGroupElementCreate();
    f->type = el->type;
    SMGroupElement *r = (SMGroupElement *)SMElementCopy((SMElement *)el);
    
    SMElement *fn = NULL, *rn = NULL;
    for (CTInteger i = -1+el->elems->count; i >= 0 ; i--)
    {
        SMElementFactorize(el->elems->data[i], &fn, &rn, x);
        
        if (fn)
        {
            SMArrayAddElemAtTop(f->elems, fn);
            
            SMElementDealloc(r->elems->data[i]);
            if (rn) {
                r->elems->data[i] = rn;
            }
            else
                SMArrayRemoveElemAtIndex(r->elems, i);
                
        }
        else if (rn)
            SMElementDealloc(rn);
    }
    
    if (!f->elems->count) {
        SMElementDealloc((SMElement *)f);
        *_f = NULL;
    }
    else if (f->elems->count == 1)
    {
        *_f = f->elems->data[0];
        if (f->type & SMElementTypeNegativeMask)
            (*_f)->type ^= SMElementTypeNegativeMask;
        SMArrayRemoveElemAtIndex(f->elems, 0);
        SMElementDealloc((SMElement *)f);
    }
    else
        *_f = (SMElement *)f;
    
    if (!r->elems->count) {
        SMElementDealloc((SMElement *)r);
        *_r = NULL;
    }
    else if (r->elems->count == 1)
    {
        *_r = r->elems->data[0];
        if (r->type & SMElementTypeNegativeMask)
            (*_r)->type ^= SMElementTypeNegativeMask;
        SMArrayRemoveElemAtIndex(r->elems, 0);
        SMElementDealloc((SMElement *)r);
    }
    else
        *_r = (SMElement *)r;
    
    return CTSuccess;
}

CTResult SMDivElementFactorize(SMGroupElement *el, SMElement **_f, SMElement **_r, SMSymbolElement *x)
{
    SMElement *fn = NULL, *rn = NULL;
    SMGroupElement *f = NULL;
    SMGroupElement *r = NULL;
    SMElementFactorize(el->elems->data[0], &fn, &rn, x);
    
    if (fn)
    {
        f = SMGroupElementCreate();
        f->type = el->type;
        SMArrayAddElem(f->elems, fn);
        for (CTUInteger i = 1; i < el->elems->count; i++) {
            SMArrayAddElem(f->elems, SMElementCopy(el->elems->data[i]));
        }
    }
    
    if (rn)
    {
        r = SMGroupElementCreate();
        r->type = el->type;
        SMArrayAddElem(r->elems, rn);
        for (CTUInteger i = 1; i < el->elems->count; i++) {
            SMArrayAddElem(r->elems, SMElementCopy(el->elems->data[i]));
        }
    }
    
    *_f = (SMElement *)f;
    *_r = (SMElement *)r;
    
    return CTSuccess;
}

CTResult SMElementFactorize(SMElement *el, SMElement **f, SMElement **r, SMSymbolElement *x)
{
    SMElementType type = el->type & ~SMElementTypeNegativeMask;
    
    if (type == SMElementTypeSymbol)
        return SMSymbolElementFactorize((SMSymbolElement *)el, f, r, x);
    else if (type == SMElementTypeMult)
        return SMMultElementFactorize((SMGroupElement *)el, f, r, x);
    else if (type == SMElementTypeDiv)
        return SMDivElementFactorize((SMGroupElement *)el, f, r, x);
    else if (type == SMElementTypeAdd)
        return SMAddElementFactorize((SMGroupElement *)el, f, r, x);
    
    *f = NULL;
    *r = SMElementCopy(el);
    
    return CTSuccess;
}

CTResult SMElementForcedSimpleCoeffs(SMElement *el, SMArray *c, SMArray *t, const SMArray *x)
{
    SMElement *f, *r = NULL;
    el = SMElementCopy(el);
    for (CTUInteger i = 0; i < x->count; i++)
    {
        SMSymbolElement * x_sym = x->data[i];
        if (x_sym->type != SMElementTypeSymbol)
        {
            printf("%s: %ld not sym\n", __func__, i);
            continue;
        }
        
        printf("    --> Factorizing for '%s'\n", x_sym->name->data);
        
        SMElementFactorize(el, &f, &r, x_sym);
        
        SMElementDealloc(el);
        el = r;
        
        // force zero coefficient
        if (!f) f = (SMElement *)SMNumberElementCreateWithValue(0);
        
        SMArrayAddElem(t, x_sym);
        SMArrayAddElem(c, f);
    }
    
    // force zero coefficient
    if (!el) el = (SMElement *)SMNumberElementCreateWithValue(0);

    SMArrayAddElem(t, SMNumberElementCreateWithValue(1));
    SMArrayAddElem(c, el);
    
    return CTSuccess;
}

CTResult SMElementSimpleCoeffs(SMElement *el, SMArray *c, SMArray *t, const SMArray *x)
{
    SMElement *f, *r = NULL;
    el = SMElementCopy(el);
    for (CTUInteger i = 0; i < x->count; i++)
    {
        SMSymbolElement * x_sym = x->data[i];
        if (x_sym->type != SMElementTypeSymbol)
        {
            printf("%s: %ld not sym\n", __func__, i);
            continue;
        }
        
        printf("    --> Factorizing for '%s'\n", x_sym->name->data);
        
        
        SMElementFactorize(el, &f, &r, x_sym);
        
        SMElementDealloc(el);
        el = r;
        
        if (f)
        {
            SMArrayAddElem(t, x_sym);
            SMArrayAddElem(c, f);
        }
//        else
//        {
//            printf("f?\n");
//            
//        }
    }
    
    if (el)
    {
        SMArrayAddElem(t, SMNumberElementCreateWithValue(1));
        SMArrayAddElem(c, el);
    }
    
    return CTSuccess;
}

#pragma mark • SMSymbolElement

SMSymbolElement *  SMSymbolElementCreateWithName(SMString *name) {
    SMSymbolElement *obj = malloc(sizeof(SMSymbolElement));
    if (obj) {
        obj->type = SMElementTypeSymbol;
        obj->name = name;
    }
    
    return obj;
}

void SMSymbolElementDealloc(SMSymbolElement *obj) {
    if (obj) {
//        SMStringDealloc(obj->name);
        
        free(obj);
    }
}


#pragma mark • SNumberElement

SMNumberElement * SMNumberElementCreateWithValue(double val) {
    SMNumberElement * obj = malloc(sizeof(SMNumberElement));
    if (obj) {
        obj->type = SMElementTypeNumber;
        obj->value = val;
    }
    
    return obj;
}

void SMNumberElementDealloc(SMNumberElement *obj) {
    free(obj);
}


#pragma mark • SMGroupElement

SMGroupElement * SMGroupElementCreate() {
    SMGroupElement * obj = malloc(sizeof(SMGroupElement));
    if (obj) {
        obj->type = SMElementTypeGroup;
        if (!(obj->elems = SMArrayCreate())) {
            SMGroupElementDealloc(obj);
            return NULL;
        }
    }
    
    return obj;
}

void SMGroupElementDealloc(SMGroupElement *obj) {
    
    for (CTUInteger i = 0; i < obj->elems->count; i++)
        SMElementDealloc(obj->elems->data[i]);
    
    SMArrayDealloc(obj->elems);
    
    free(obj);
}


#pragma mark • SMMatrixElement

SMMatrixElement * SMMatrixElementCreate() {
    SMMatrixElement *obj = malloc(sizeof(SMMatrixElement));
    if (obj) {
        obj->type = SMElementTypeMatrix;
        obj->rows = obj->cols = 0;
        if (!(obj->elems = SMArrayCreate())) {
            SMMatrixElementDealloc(obj);
            return NULL;
        }
    }
    
    return obj;
}
void SMMatrixElementDealloc(SMMatrixElement *obj) {
//    SMArrayDealloc(obj->elems);
    SMGroupElementDealloc((SMGroupElement *)obj);
    
//    free(obj);
}


#pragma mark • SMFunElement

SMFunElement * SMFunElementCreateWithName(SMString *name) {
    SMFunElement * obj = malloc(sizeof(SMFunElement));
    if (obj) {
        obj->type = SMElementTypeFun;
        obj->name = name;
        if (!(obj->elems = SMArrayCreate())) {
            SMFunElementDealloc(obj);
            return NULL;
        }
    }
    
    return obj;
}

void SMFunElementDealloc(SMFunElement *obj) {
//    SMStringDealloc(obj->name);
//    SMArrayDealloc(obj->elems);
    
    SMGroupElementDealloc((SMGroupElement *)obj);
    
//    free(obj);
}


#pragma mark - SMVar

SMVar * SMVarCreate() {
    SMVar * obj = malloc(sizeof(SMVar));
    if (obj) {
        obj->sym  = NULL;
        obj->elem = NULL;
    }
    
    return obj;
}

void SMVarDealloc(SMVar *obj) {
    if (obj) {
        SMSymbolElementDealloc(obj->sym);
        SMElementDealloc(obj->elem);
        
        free(obj);
    }
}

CTResult SMVarRepresenatiion(SMVar *var, SMRepresentation rep, SMString *str)
{
    CTResult res;
    if (rep == SMCRepresentation &&
        CTSuccess != (res = SMStringAppendFormat(str, "double ")))
        return res;
    
    if (CTSuccess != (res = SMElementToRepresentation((SMElement *)var->sym, rep, str)))
        return res;
    
    if (rep == SMCRepresentation &&
        (var->elem->type & ~SMElementTypeNegativeMask) == SMElementTypeMatrix)
    {
        SMMatrixElement *el = (SMMatrixElement *)var->elem;
        
        if (CTSuccess != (res = SMStringAppendFormat(str, "[%d][%d]", el->rows, el->cols)))
            return res;
    }
    
    if (CTSuccess != (res = SMStringAppendFormat(str, " = ")) ||
        CTSuccess != (res = SMElementToRepresentation(var->elem, rep, str)) ||
        CTSuccess != (res = SMStringAppendChar(str, ';')))
        return res;
    
    return CTSuccess;
}


#pragma mark - SMContext

SMContext * SMContextCreate() {
    SMContext * obj = malloc(sizeof(SMContext));
    if (obj) {
        obj->vars     = NULL;
        obj->infoVars = NULL;
        obj->symNames = NULL;
        if (!(obj->vars     = SMArrayCreate()) ||
            !(obj->infoVars = SMArrayCreate()) ||
            !(obj->symNames = SMArrayCreate())) {
            SMContextDealloc(obj);
            return NULL;
        }
    }
    
    return obj;
}

void SMContextDealloc(SMContext * obj) {
    if (obj) {
        
        for (size_t i = 0; i < obj->vars->count; i++)
            SMVarDealloc((SMVar *)obj->vars->data[i]);
        
        for (size_t i = 0; i < obj->infoVars->count; i++)
            SMVarDealloc((SMVar *)obj->infoVars->data[i]);
        
        for (size_t i = 0; i < obj->symNames->count; i++)
            SMStringDealloc((SMString *)obj->symNames->data[i]);
        
        SMArrayDealloc(obj->vars);
        SMArrayDealloc(obj->infoVars);
        SMArrayDealloc(obj->symNames);
        
        free(obj);
    }
}

int __SMStringComp(SMString *str1, SMString *str2)
{
    size_t n = (str1)->len < (str2)->len ? (str1)->len:(str2)->len;
    
    int res = strncmp((str1)->data, (str2)->data, n);
    
    if (!res)
    {
        if ((str1)->len < (str2)->len)
            return -1;
        else if ((str1)->len > (str2)->len)
            return 1;
    }
    
    return res;
}

SMSubString SMGetMatrix(SMSubString subStr)
{
    SMSubString tmp;
    if (!(tmp = SMGetMaximaMatrix(subStr)).len &&
        !(tmp = SMGetOctaveMatrix(subStr)).len)
        tmp = subStr;
    
    return tmp;
}

SMSubString SMGetOctaveMatrix(SMSubString subStr)
{
    SMSubString tmp;
    tmp = SMSubStringScanUpBeforeFirstOccurenceOfStr(&subStr, "# name: flat");
    if (!subStr.len) subStr = tmp;
    tmp = SMSubStringScanUpBeforeFirstOccurenceOfStr(&subStr, "Matrix");
    if (!subStr.len)
//        subStr = tmp;
        return (SMSubString){NULL, 0};
    else
    {
        SMSubStringScanUpBeforeFirstOccurenceOfChar(&subStr, '(');
        subStr = SMSubStringScanGroup(&subStr);
    }
    
    return subStr;
}

SMSubString SMGetMaximaMatrix(SMSubString subStr)
{
    SMSubString tmp;
    tmp = SMSubStringScanUpBeforeFirstOccurenceOfStr(&subStr, "matrix");
    if (!subStr.len)
//        subStr = tmp;
        return (SMSubString){NULL, 0};
    else
    {
        SMSubStringScanUpBeforeFirstOccurenceOfChar(&subStr, '(');
        subStr = SMSubStringScanGroup(&subStr);
        // FIXME: better handling of maxima matrix
        subStr.data--;
        subStr.data[0] = '[';
        subStr.len++;
        subStr.data[subStr.len] = ']';
        subStr.len++;
    }
    
    return subStr;
}

SMString * SMContextStringForStr(SMContext * obj, const char *name)
{
//    CTUInteger loc;
//    // Not nice
//        SMString tmpstr = {0};
//        tmpstr.data = (char *)name;
//    tmpstr.len = strlen(name);
//    if (!SMArraySearch(obj->symNames, &loc, &tmpstr, (int (*)(void *, void *))__SMStringComp))
//        SMArrayInsertElemAtIndex(obj->symNames, loc, SMStringCreateWithString(name));
//    
//    return obj->symNames->data[loc];
    
    return SMContextStringForSubString(obj, (SMSubString){(char *)name, strlen(name)});
}

SMString * SMContextStringForSubString(SMContext * obj, SMSubString name)
{
    CTUInteger loc;
    // Not nice
        SMString tmpstr = {0};
        tmpstr.data = (char *)name.data;
        tmpstr.len = name.len;
//        SMString *str = &tmpstr;
    
    if (!SMArraySearch(obj->symNames, &loc, &tmpstr, (int (*)(void *, void *))__SMStringComp))
        SMArrayInsertElemAtIndex(obj->symNames, loc, SMStringCreateWithSubString(name));
    
    return obj->symNames->data[loc];
}

SMElement * SMContextScanElement(SMContext * obj, SMSubString data);
SMElement * SMContextScanMatrix(SMContext * obj, SMSubString data);
CTResult    SMContextScanFunctionParams(SMContext * obj, SMSubString data, SMArray *params);
SMElement * SMContextScanFirstElement(SMContext * obj, SMSubString *data);
SMElement * SMContextScanSimpleElement(SMContext * obj, SMSubString data);

SMElement * SMContextScanGroupElement(SMContext * obj, SMSubString *data, SMElementType *type, SMElement *firstElem);

SMElementType SMScanType(SMSubString *data);

SMElement * SMContextScanSimpleElement(SMContext * obj, SMSubString data)
{
    SMSubStringRemoveSurroundingWhiteSpace(&data);
    SMString *str = SMStringCreateWithSubString(data);
    SMElement *elem;
    if (data.data[0] >= '0' && data.data[0] <= '9') {
        elem = (SMElement *)SMNumberElementCreateWithValue(atof(str->data));
//        SMStringDealloc(str);
    }
    else
        elem = (SMElement *)SMSymbolElementCreateWithName(SMContextStringForSubString(obj, (SMSubString){str->data, str->len}));
    
    SMStringDealloc(str);
    
    return elem;
}

SMElementType SMScanType(SMSubString *data)
{
    SMSubStringRemoveSurroundingWhiteSpace(data);
    
    if (!data->len) return SMElementTypeUknown;
    
    char c = data->data[0];
    SMElementType type;
    if (c == '^')
        type = SMElementTypePow;
    else if (c == '*' && data->len > 1 && data->data[1] == '*') {
        type = SMElementTypePow;
        data->data++;
        data->len--;
    }
    else if (c == '*')
        type = SMElementTypeMult;
    else if (c == '/')
        type = SMElementTypeDiv;
    else if (c == '+')
        type = SMElementTypeAdd;
    else if (c == '-')
        type = (SMElementTypeAdd | SMElementTypeNegativeMask);
    else {
        return SMElementTypeUknown;
    }
    
    data->data++;
    data->len--;
    
    return type;
}

SMElement * SMContextScanGroupElement(SMContext * obj, SMSubString *data, SMElementType *type, SMElement *firstElem)
{
    if (*type == SMElementTypeUknown)
        *type = SMElementTypeAdd; // Lowest order
    
    SMGroupElement * group = SMGroupElementCreate();
    group->type = *type;
    
    SMElementType mask = 0;
    
    if (!firstElem)
    {
        // Chech for negative mask
        *type = SMScanType(data);
        firstElem = SMContextScanFirstElement(obj, data);
        if (*type & SMElementTypeNegativeMask)
            firstElem->type |= SMElementTypeNegativeMask;
//    if (type == SMElementTypeUknown)
        *type = SMScanType(data); // TODO: mind uknown
    }
    
    while (firstElem)
    {
        if (type == SMElementTypeUknown && data->len) {
            SMGroupElementDealloc(group);
            return NULL;
        }
        else if ((*type & ~SMElementTypeNegativeMask) < (group->type & ~SMElementTypeNegativeMask)) // Higher order
        {
            firstElem = SMContextScanGroupElement(obj, data, type, firstElem);
        }
        else if ((*type & ~SMElementTypeNegativeMask) > (group->type & ~SMElementTypeNegativeMask)) // Lower order
        {
            if (!group->elems->count) {
                firstElem->type |= (group->type & SMElementTypeNegativeMask);
                SMGroupElementDealloc(group);
            }
            else {
                SMArrayAddElem(group->elems, firstElem);
                firstElem = (SMElement *)group;
            }
            
            return firstElem;
        }
        
        if (((*type & ~SMElementTypeNegativeMask) == (group->type & ~SMElementTypeNegativeMask)) || !data->len)
        {
            firstElem->type |= mask;
            SMArrayAddElem(group->elems, firstElem);
            
            if (data->len) { // TODO: better
                firstElem = SMContextScanFirstElement(obj, data);
                mask      = *type & SMElementTypeNegativeMask;
                *type     = SMScanType(data);
            }
            else {
                firstElem = NULL;
                *type     = SMElementTypeUknown;
            }
        }
    }
    
    if (group->elems->count == 1) {
        firstElem = group->elems->data[0];
        group->elems->count = 0;
        firstElem->type |= (group->type & SMElementTypeNegativeMask);
        SMGroupElementDealloc(group);
    }
    else {
        firstElem = (SMElement *)group;
    }
    
    return firstElem;
}

SMElement * SMContextScanFirstElement(SMContext * obj, SMSubString *data)
{
    SMElement *currElem = NULL;
    
    SMSubString tmp = SMSubStringScanUpBeforeFirstCharInStr(data, "([*/^+-");
    
    SMSubStringRemoveSurroundingWhiteSpace(&tmp);
    
    // Scientific constant
    if (tmp.len && data->len &&
        isdigit(tmp.data[0]) &&
        (tmp.data[tmp.len-1] == 'e' || tmp.data[tmp.len-1] == 'E') &&
        (data->data[0] == '-' ||
         data->data[0] == '+' ))
    {
        data->data++;
        data->len--;
//        tmp.len++;
        SMSubString tmp2 = SMSubStringScanUpBeforeFirstCharInStr(data, "([*/^+-");
        
        tmp.len += tmp2.len + 1;
    }
    
    // Get the first element
    if (data->len && data->data[0] == '(')
    {
        if (tmp.len) // function
        {
            SMString     *name = SMContextStringForSubString(obj, tmp);//SMStringCreateWithSubString(tmp);
            SMFunElement *elem = SMFunElementCreateWithName(name);
            
            tmp = SMSubStringScanGroup(data);
            SMContextScanFunctionParams(obj, tmp, elem->elems);
            
            currElem = (SMElement *)elem;
        }
        else // Group
        {
            tmp = SMSubStringScanGroup(data);
            SMElement * elem = SMContextScanElement(obj, tmp);
            
            currElem = (SMElement *)elem;
        }
        
        tmp = SMSubStringScanUpBeforeFirstCharInStr(data, "*/^+-");
    }
    else if (data->len && data->data[0] == '[')
    {
        if (tmp.len) // sub matrix
        {
            SMString     *name = SMContextStringForSubString(obj, tmp);//SMStringCreateWithSubString(tmp);
            SMFunElement *elem = SMFunElementCreateWithName(name);
            elem->type = SMElementTypeSubMatrix;
            
            tmp = SMSubStringScanGroup(data);
            SMContextScanFunctionParams(obj, tmp, elem->elems);
            
            currElem = (SMElement *)elem;
        }
        else // Matrix
        {
            tmp = SMSubStringScanGroup(data);
            SMElement * elem = SMContextScanMatrix(obj, tmp);
            
            currElem = (SMElement *)elem;
        }
        
        tmp = SMSubStringScanUpBeforeFirstCharInStr(data, "*/^+-");
    }
    else if (!tmp.len || !(currElem = SMContextScanSimpleElement(obj, tmp)))
        return NULL;

    return currElem;
}

// TODO: Allow multiple params
CTResult SMContextScanFunctionParams(SMContext * obj, SMSubString data, SMArray *params)
{
    SMElement *elem = SMContextScanElement(obj, data);
    return SMArrayAddElem(params, elem);
}

SMElement * SMContextScanElement(SMContext * obj, SMSubString data)
{
    SMElementType type = SMElementTypeUknown;
    
    return (SMElement *)SMContextScanGroupElement(obj, &data, &type, NULL);
}


CTResult SMContextScanSubMatrix(SMContext * obj, SMMatrixElement *matEl, SMSubString data)
{
    SMSubString tmp;
    
    while (data.len)
    {
        tmp = SMSubStringScanUpBeforeFirstUngroupedCharInStr(&data, "[,\n");
        
        if (data.len)
        {
            if (*data.data == '[')
            {
                tmp = SMSubStringScanGroup(&data);
                SMContextScanSubMatrix(obj, matEl, tmp);
            }
            else {
                
                SMSubStringRemoveSurroundingWhiteSpace(&tmp);
                if (tmp.len)
                {
                    SMElement *elem = SMContextScanElement(obj, tmp);
                    SMArrayAddElem(matEl->elems, elem);
                    if (matEl->rows == 1) matEl->cols++;
                }
                
                if (*data.data == ',') {
                    data.data++;
                    data.len--;
                    // ??
                }
                else if (*data.data == '\n') {
                    if (matEl->elems->count && matEl->rows * matEl->cols == matEl->elems->count) {
                        matEl->rows++;
                    }
                    data.data++;
                    data.len--;
                    
                }
                else
                {
                    data.data++;
                    data.len--;
                    // ??
                }
            }
        }
        else
        {
            SMSubStringRemoveSurroundingWhiteSpace(&tmp);
            if (tmp.len)
            {
                SMElement *elem = SMContextScanElement(obj, tmp);
                SMArrayAddElem(matEl->elems, elem);
                if (matEl->rows == 1) matEl->cols++;
            }
        }
        
        
    }
    
    return CTSuccess;
}

// TODO: Check
SMElement * SMContextScanMatrix(SMContext * obj, SMSubString data) {
    
    SMMatrixElement *matEl = SMMatrixElementCreate();
    matEl->rows = 1;
    
    CTResult res = SMContextScanSubMatrix(obj, matEl, data);
    
    if (matEl->rows * matEl->cols > matEl->elems->count) {
        matEl->rows--;
    }
    
    if (matEl->elems->count == 1) {
        SMElement *el = matEl->elems->data[0];
        SMMatrixElementDealloc(matEl);
        return el;
    }
    
    return (SMElement *)matEl;
}

SMVar * SMContextScanVar(SMContext * obj, SMSubString data) {
    SMVar *var = SMVarCreate();
    if (!var) return NULL;
    
    SMSubString tmp = SMSubStringScanUpBeforeFirstCharInStr(&data, "=");
    
    if (data.len) {
        var->sym = (SMSymbolElement *)SMContextScanSimpleElement(obj, tmp);
        data.data++;
        data.len--;
        
        if (var->sym->type != SMElementTypeSymbol) {
            SMElementDealloc((SMElement *)var->sym);
            var->sym = NULL;
        }
        
        tmp = data;
    }

    if (!var->sym) var->sym = SMSymbolElementCreateWithName(SMContextStringForStr(obj, "var"));
//    if (!var->sym) var->sym = SMSymbolElementCreateWithName(SMStringCreateWithString("var"));
    var->elem = SMContextScanElement(obj, tmp);
    
    return var;
}

CTResult SMContextAddVar(SMContext * obj, SMVar *var)
{
    if (!var->sym || !var->elem)
        return CTError;
    
    CTUInteger idx = obj->vars->count;
    
    for (CTInteger i = obj->vars->count -1; i >= 0 ; i--) {
        SMVar *ivar = obj->vars->data[i];
        if (SMElementReplace((SMElement **)&(ivar->elem), var->elem, (SMElement *)var->sym))
            idx = i;
    }
    
    return SMArrayInsertElemAtIndex(obj->vars, idx, var);
}

CTResult SMContextGetUknownSymbols(SMContext * obj, SMArray *elems)
{
    CTResult res;
    if (CTSuccess != (res = SMContextGetElementsOfType(obj, SMElementTypeSymbol, elems)))
        return res;
    
    for (size_t i = 0; i < obj->vars->count; i++)
    {
        CTUInteger idx = SMElementArrayIndexOfElement(elems, (SMElement *)((SMVar *)obj->vars->data[i])->sym);
        if (idx != CTNotFound)
            SMArrayRemoveElemAtIndex(elems, idx);
    }
    
    return CTSuccess;
}

CTResult SMContextGetElementsOfType(SMContext * obj, SMElementType type, SMArray *elems)
{
    CTResult res;
    
    for (size_t i = 0; i < obj->vars->count; i++) {
        if (CTSuccess != (res = SMElementGetElementsOfType(((SMVar *)obj->vars->data[i])->elem, type, elems)))
            return res;
    }

    return CTSuccess;
}

CTResult SMContextRepresenation(SMContext * obj, SMRepresentation rep, SMString *str)
{
    CTResult res = CTSuccess;
    for (size_t i = 0; i < obj->vars->count; i++) {
        if (CTSuccess != (res = SMStringAppendFormat(str, "    ")) ||
            CTSuccess != (res = SMVarRepresenatiion(obj->vars->data[i], rep, str))||
            CTSuccess != (res = SMStringAppendChar(str, '\n')))
            return res;
    }
    
    return res;
}

CTResult SMContextLoadFromString(SMContext * obj, SMString *str)
{
    SMSubString subStr = {str->data, str->len};
    
    return SMContextLoadFromSubString(obj, subStr);
}

CTResult SMContextLoadFromSubString(SMContext * obj, SMSubString subStr)
{
    SMSubString tmp;
    SMSubStringRemoveSurroundingWhiteSpace(&subStr);
    do {
        tmp = SMSubStringScanUpBeforeFirstUngroupedCharInStr(&subStr, "\n");
        SMSubStringRemoveSurroundingWhiteSpace(&subStr);
        SMVar *var = SMContextScanVar(obj, tmp);
        if (var) SMArrayAddElem(obj->vars, var);
    } while (subStr.len);
    
    return CTSuccess;
}

CTResult SMContextLoadInfoFromSubString(SMContext * obj, SMSubString subStr)
{
    SMSubString tmp;
    
    do {
        tmp = SMSubStringScanUpBeforeFirstUngroupedCharInStr(&subStr, "\n");
        SMSubStringRemoveSurroundingWhiteSpace(&subStr);
        SMVar *var = SMContextScanVar(obj, tmp);
        if (var) SMArrayAddElem(obj->infoVars, var);
    } while (subStr.len);
    
    return CTSuccess;
}

CTResult SMContextLoadInfoFromString(SMContext * obj, SMString *str)
{
    SMSubString subStr = {str->data, str->len};
    SMSubString tmp;
    
    do {
        tmp = SMSubStringScanUpBeforeFirstUngroupedCharInStr(&subStr, "\n");
        SMSubStringRemoveSurroundingWhiteSpace(&subStr);
        SMVar *var = SMContextScanVar(obj, tmp);
        if (var) SMArrayAddElem(obj->infoVars, var);
    } while (subStr.len);
    
    return CTSuccess;
}


