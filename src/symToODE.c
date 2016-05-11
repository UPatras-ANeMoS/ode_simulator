//
//  main.c
//  SymToODE
//
//  Created by Kostis Giannousakis on 23/01/16.
//  Copyright © 2016 Kostis Giannousakis. All rights reserved.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Symbolic.h"
#include <time.h>
#include <math.h>

//#define USE_EIGEN

clock_t __stime, __etime;
double secs, mins, hrs;
# define MEASURE_BEGIN() __stime = clock();
# define MEASURE_END()                                  \
            __etime = clock();                            \
            secs = (__etime-__stime)/(double)CLOCKS_PER_SEC;\
            mins = (int)(secs/60);                      \
            secs -= 60*mins;                            \
            hrs = (int)(mins/60);                       \
            mins -= 60*hrs;                             \
            printf("    Time: %gh %gm %gs\n", hrs, mins, secs);
# define MEASURE(code) MEASURE_BEGIN(); code MEASURE_END()

int main(int argc, const char * argv[]) {
    
    if (argc < 3) {
        fprintf(stderr, "Syntax: %s dataFolder outFolder\n", argv[0]);
        return -1;
    }
    
    char PFile[255];
    char DFile[255];
    char WFile[255];
    char XFile[255];
    char UFile[255];
    
    char FcFile[255];
    char FhFile[255];

    bool needSlash = (*argv[1] && argv[1][strlen(argv[1])-1] != '/');
    
    sprintf(PFile, "%s%s%s", argv[1], needSlash ? "/":"", "P.txt");
    sprintf(DFile, "%s%s%s", argv[1], needSlash ? "/":"", "D.txt");
    sprintf(WFile, "%s%s%s", argv[1], needSlash ? "/":"", "W.txt");
    sprintf(XFile, "%s%s%s", argv[1], needSlash ? "/":"", "X.txt");
    sprintf(UFile, "%s%s%s", argv[1], needSlash ? "/":"", "U.txt");

    needSlash = (*argv[2] && argv[2][strlen(argv[2])-1] != '/');
# ifdef USE_EIGEN
    sprintf(FcFile, "%s%s%s", argv[2], needSlash ? "/":"", "odefun.cpp");
# else
    sprintf(FcFile, "%s%s%s", argv[2], needSlash ? "/":"", "odefun.c");
# endif
    sprintf(FhFile, "%s%s%s", argv[2], needSlash ? "/":"", "odefun.h");
    
    SMContext *ctxt = SMContextCreate();
    SMString  *str  = SMStringCreate();
    SMSubString subStr;

    // X file
    CTUInteger iX = ctxt->infoVars->count;
    
    printf("--> Loading file: %s\n", XFile);
    MEASURE_BEGIN()
    SMStringEmpty(str);
    SMStringLoadContentsOfFile(str, XFile);
    subStr = (SMSubString){str->data, str->len};
    subStr = SMGetMatrix(subStr);
    SMContextLoadInfoFromSubString(ctxt, subStr);
    MEASURE_END()
    
    SMVar *Xvar;
    if ((ctxt->infoVars->count - iX) != 1 ||
        (Xvar = ctxt->infoVars->data[iX])->elem->type != SMElementTypeMatrix)
    {
        fprintf(stderr, "Error: X must be an array of symbols\n");
        return -1;
    }
    Xvar->sym = SMSymbolElementCreateWithName(SMStringCreateWithString("X"));
    
    SMMatrixElement *Xel = (SMMatrixElement *)Xvar->elem;
    
    // U file
    CTUInteger iU = ctxt->infoVars->count;
    
    printf("--> Loading file: %s\n", UFile);
    MEASURE_BEGIN()
    SMStringEmpty(str);
    SMStringLoadContentsOfFile(str, UFile);
    subStr = (SMSubString){str->data, str->len};
    subStr = SMGetMatrix(subStr);
    SMContextLoadInfoFromSubString(ctxt, subStr);
    MEASURE_END()
    
    SMVar *Uvar;
    if ((ctxt->infoVars->count - iU) != 1 ||
        (Uvar = ctxt->infoVars->data[iU])->elem->type != SMElementTypeMatrix)
    {
        fprintf(stderr, "Error: U must be an array of symbols\n");
        return -1;
    }
    Uvar->sym = SMSymbolElementCreateWithName(SMStringCreateWithString("U"));
    
    SMMatrixElement *Uel = (SMMatrixElement *)Uvar->elem;
    
    
    CTUInteger n = Xel->elems->count;
    CTUInteger m = Uel->elems->count;
    
    printf("--> Loading file: %s\n", PFile);
    MEASURE(
        SMStringEmpty(str);
        if(CTSuccess == SMStringLoadContentsOfFile(str, PFile))
       		SMContextLoadFromString(ctxt, str);
    )
    
    // D file
    CTUInteger iD = ctxt->vars->count;
    
    printf("--> Loading file: %s\n", DFile);
    MEASURE_BEGIN()
    SMStringEmpty(str);
    SMStringLoadContentsOfFile(str, DFile);
    subStr = (SMSubString){str->data, str->len};
    subStr = SMGetMatrix(subStr);
    SMContextLoadFromSubString(ctxt, subStr);
    MEASURE_END()
    
    SMVar *Dvar;
    if ((ctxt->vars->count - iD) != 1 ||
        (Dvar = ctxt->vars->data[iD])->elem->type != SMElementTypeMatrix)
    {
        fprintf(stderr, "Error: D must be a square matrix\n");
        return -1;
    }
    Dvar->sym = SMSymbolElementCreateWithName(SMStringCreateWithString("D"));
    
    SMMatrixElement *Dmat = (SMMatrixElement *)Dvar->elem;
    if (Dmat->rows == 1) // Maxima fix
        Dmat->rows = Dmat->cols = (CTUInteger)sqrt(Dmat->cols);
    
    // W file
    CTUInteger iW = ctxt->vars->count;
    
    printf("--> Loading file: %s\n", WFile);
    MEASURE_BEGIN();
    SMStringEmpty(str);
    SMStringLoadContentsOfFile(str, WFile);
    subStr = (SMSubString){str->data, str->len};
    subStr = SMGetMatrix(subStr);
    SMContextLoadFromSubString(ctxt, subStr);
    MEASURE_END();
    
    SMVar *Wvar;
    if ((ctxt->vars->count - iW) != 1 ||
        (Wvar = ctxt->vars->data[iW])->elem->type != SMElementTypeMatrix)
    {
        fprintf(stderr, "Error: W must be an array\n");
        return -1;
    }
    Wvar->sym = SMSymbolElementCreateWithName(SMStringCreateWithString("W"));
    
    SMMatrixElement *Wmat = (SMMatrixElement *)Wvar->elem;
    if (Wmat->rows == 1) { // Maxima fix
        Wmat->rows = Wmat->cols;
        Wmat->cols = 1;
    }
    
    // Replace simple function with variables
    SMGroupElement *gel = SMGroupElementCreate();

    printf("--> Finding functions\n");
    MEASURE(
        SMContextGetElementsOfType(ctxt, SMElementTypeFun, gel->elems);
    )
    
    printf("--> Replacing univariable functions with variables\n");
    MEASURE_BEGIN();
    for (CTUInteger i = 0; i < gel->elems->count; i++)
    {
        SMFunElement *el = gel->elems->data[i];
        
        if (el->elems->count != 1) continue;
        
        if ((((SMElement *)el->elems->data[0])->type) != SMElementTypeSymbol)
            continue;
        
        SMSymbolElement *sym = el->elems->data[0];
        SMString *name = SMStringCreate();
        SMStringAppendFormat(name, "%c_%s", *el->name->data, sym->name->data);
        SMVar *var = SMVarCreate();
        var->sym  = SMSymbolElementCreateWithName(name);
        var->elem = (SMElement *)el;
        
        SMContextAddVar(ctxt, var);
    }
    MEASURE_END();
    
    // Get uknown symbols from info
    gel->elems->count = 0;
    printf("--> Looking for uknown symbols\n");
    MEASURE(
        SMContextGetUknownSymbols(ctxt, gel->elems);
    )
    
    printf("--> Creating variables for uknown symbols based on input\n");
    MEASURE_BEGIN();
    for (CTUInteger i = 0; i < gel->elems->count; i++)
    {
        SMSymbolElement *el = gel->elems->data[i];
        int found = 0;
        
        for (CTUInteger j = 0; !found && j < Xel->elems->count; j++)
        {
            if (!SMElementCompare((SMElement *)el, Xel->elems->data[j]))
            {
                SMVar *var = SMVarCreate();
                SMFunElement *subM = SMFunElementCreateWithName(SMStringCreateWithString(Xvar->sym->name->data));
                subM->type = SMElementTypeSubMatrix;
                SMArrayAddElem(subM->elems, SMNumberElementCreateWithValue(j));
                var->sym = (SMSymbolElement *)SMElementCopy(Xel->elems->data[j]);
                var->elem = (SMElement *)subM;
                
                SMArrayInsertElemAtIndex(ctxt->vars, 0, var);
                
                found = 1;
            }
        }
        
        for (CTUInteger j = 0; !found && j < Uel->elems->count; j++)
        {
            if (!SMElementCompare((SMElement *)el, Uel->elems->data[j]))
            {
                SMVar *var = SMVarCreate();
                SMFunElement *subM = SMFunElementCreateWithName(SMStringCreateWithString(Uvar->sym->name->data));
                subM->type = SMElementTypeSubMatrix;
                SMArrayAddElem(subM->elems, SMNumberElementCreateWithValue(j));
                var->sym = (SMSymbolElement *)SMElementCopy(Uel->elems->data[j]);
                var->elem = (SMElement *)subM;
                
                SMArrayInsertElemAtIndex(ctxt->vars, 0, var);
                
                found = 1;
            }
        }
    }
    MEASURE_END();
    
    gel->elems->count = 0;
    printf("--> Looking for uknown symbols once more\n");
    MEASURE(
        SMContextGetUknownSymbols(ctxt, gel->elems);
    )
    
    if (gel->elems->count) {
        printf("--> Uknown Symbols: ");
        SMGroupElementPrint(gel, ", ");
        printf("\n");
    }
    
    // Create the header file
    SMString *crep = SMStringCreate();
    
    printf("--> Saving '%s'\n", FhFile);
# ifdef USE_EIGEN
    SMStringAppendFormat(crep,
        "#include <iostream>\n"
        "\n"
        "const unsigned int n = %d;\n"
        "const unsigned int m = %d;\n"
        "void odefun(const double X[%ld] , double dX[%ld], double U[%ld] , double t);\n", n ,m, n, n, m);
//        "void odefun(const double X[%ld] , double dX[%ld], double U[%ld] , double t);\n", n, n, m);
# else
    SMStringAppendFormat(crep,
        "#include \"CoreTypesLite.h\"\n"
        "\n"
        "const unsigned int n = %d;\n"
        "const unsigned int m = %d;\n"
        "void odefun(const double X[%ld] , double dX[%ld], double U[%ld] , double t);\n", n ,m, n, n, m);
//        "void odefun(const double X[%ld] , double dX[%ld], double U[%ld] , double t);\n", n, n, m);
# endif
    SMStringSave(crep, FhFile);
    
    // Create the c file
    printf("--> Saving '%s'\n", FcFile);
    SMStringEmpty(crep);
# ifdef USE_EIGEN
    SMStringAppendFormat(crep,
        "#include \"odefun.h\"\n"
        "#include <Eigen/Dense>\n"
        "\n"
        "using namespace Eigen;\n"
        "\n"
        "void odefun(const double X[%ld] , double dX[%ld], double U[%ld] , double t)\n"
        "{\n", n, n, m);
# else
    SMStringAppendFormat(crep,
        "#include \"odefun.h\"\n"
        "#include <math.h>\n"
        "#include \"MatrixLite.h\"\n"
        "\n"
        "void odefun(const double X[%ld] , double dX[%ld], double U[%ld] , double t)\n"
        "{\n", n, n, m);
# endif
    
    SMContextRepresenation(ctxt, SMCRepresentation, crep);
    
# ifdef USE_EIGEN
    SMStringAppendFormat(crep,  "\n"
                                "   Map<MatrixXd> Dm((double*)D,%d,%d);\n"
                                "   Map<VectorXd> Wv((double*)W,%d);\n"
                                "\n"
                                "   Wv = Dm.colPivHouseholderQr().solve(Wv);\n\n", n/2,n/2,n/2);
    for (CTUInteger i = 0, count = n/2; i < count; i++)
    {
        SMStringAppendFormat(crep, "    dX[%ld] = X[%ld];\n", 2*i, 2*i+1);
        SMStringAppendFormat(crep, "    dX[%ld] = Wv(%ld);\n", 2*i+1, i);
    }
# else
    SMStringAppendFormat(crep, "\n    mxSystemSolve((CTFloat *)D, (CTFloat *)W, %ld);\n\n", n/2);
    for (CTUInteger i = 0, count = n/2; i < count; i++)
    {
        SMStringAppendFormat(crep, "    dX[%ld] = X[%ld];\n", 2*i, 2*i+1);
        SMStringAppendFormat(crep, "    dX[%ld] = W[%ld][0];\n", 2*i+1, i);
    }
# endif
    
    
    SMStringAppendFormat(crep, "\n}\n");
    
    SMStringSave(crep, FcFile);
    
    return 0;
}
