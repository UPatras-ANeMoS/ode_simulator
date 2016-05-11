//
//  CoreTypesLite.h
//  CoreTypes
//
//  Created by Kostis Giannousakis on 06/02/16.
//  Copyright © 2016 Kostis Giannousakis. All rights reserved.
//

#ifndef CoreTypesLite_h
#define CoreTypesLite_h

#include <stdio.h>
#include <limits.h>
#include <stdbool.h>

#define CTIntegerMax    LONG_MAX

typedef long            CTInteger;
typedef unsigned long   CTUInteger;
enum {CTNotFound = CTIntegerMax};

enum _CTResult
{
    CTError   = -1,
    CTSuccess =  0
};

typedef CTInteger CTResult;

typedef double      CTFloat;
typedef long double CTDouble;

#endif /* CoreTypesLite_h */
