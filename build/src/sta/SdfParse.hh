/* A Bison parser, made by GNU Bison 3.8.2.  */

/* Bison interface for Yacc-like parsers in C

   Copyright (C) 1984, 1989-1990, 2000-2015, 2018-2021 Free Software Foundation,
   Inc.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>.  */

/* As a special exception, you may create a larger work that contains
   part or all of the Bison parser skeleton and distribute that work
   under terms of your choice, so long as that work isn't itself a
   parser generator using the skeleton or a modified version thereof
   as a parser skeleton.  Alternatively, if you modify or redistribute
   the parser skeleton itself, you may (at your option) remove this
   special exception, which will cause the skeleton and the resulting
   Bison output files to be licensed under the GNU General Public
   License without this special exception.

   This special exception was added by the Free Software Foundation in
   version 2.2 of Bison.  */

/* DO NOT RELY ON FEATURES THAT ARE NOT DOCUMENTED in the manual,
   especially those whose name start with YY_ or yy_.  They are
   private implementation details that can be changed or removed.  */

#ifndef YY_SDFPARSE_HOME_CAPTAINNOTHING_OPENROAD_BUILD_SRC_STA_SDFPARSE_HH_INCLUDED
# define YY_SDFPARSE_HOME_CAPTAINNOTHING_OPENROAD_BUILD_SRC_STA_SDFPARSE_HH_INCLUDED
/* Debug traces.  */
#ifndef YYDEBUG
# define YYDEBUG 0
#endif
#if YYDEBUG
extern int SdfParse_debug;
#endif

/* Token kinds.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
  enum yytokentype
  {
    YYEMPTY = -2,
    YYEOF = 0,                     /* "end of file"  */
    YYerror = 256,                 /* error  */
    YYUNDEF = 257,                 /* "invalid token"  */
    DELAYFILE = 258,               /* DELAYFILE  */
    SDFVERSION = 259,              /* SDFVERSION  */
    DESIGN = 260,                  /* DESIGN  */
    DATE = 261,                    /* DATE  */
    VENDOR = 262,                  /* VENDOR  */
    PROGRAM = 263,                 /* PROGRAM  */
    PVERSION = 264,                /* PVERSION  */
    DIVIDER = 265,                 /* DIVIDER  */
    VOLTAGE = 266,                 /* VOLTAGE  */
    PROCESS = 267,                 /* PROCESS  */
    TEMPERATURE = 268,             /* TEMPERATURE  */
    TIMESCALE = 269,               /* TIMESCALE  */
    CELL = 270,                    /* CELL  */
    CELLTYPE = 271,                /* CELLTYPE  */
    INSTANCE = 272,                /* INSTANCE  */
    DELAY = 273,                   /* DELAY  */
    ABSOLUTE = 274,                /* ABSOLUTE  */
    INCREMENTAL = 275,             /* INCREMENTAL  */
    INTERCONNECT = 276,            /* INTERCONNECT  */
    PORT = 277,                    /* PORT  */
    DEVICE = 278,                  /* DEVICE  */
    RETAIN = 279,                  /* RETAIN  */
    IOPATH = 280,                  /* IOPATH  */
    TIMINGCHECK = 281,             /* TIMINGCHECK  */
    SETUP = 282,                   /* SETUP  */
    HOLD = 283,                    /* HOLD  */
    SETUPHOLD = 284,               /* SETUPHOLD  */
    RECOVERY = 285,                /* RECOVERY  */
    REMOVAL = 286,                 /* REMOVAL  */
    RECREM = 287,                  /* RECREM  */
    WIDTH = 288,                   /* WIDTH  */
    PERIOD = 289,                  /* PERIOD  */
    SKEW = 290,                    /* SKEW  */
    NOCHANGE = 291,                /* NOCHANGE  */
    POSEDGE = 292,                 /* POSEDGE  */
    NEGEDGE = 293,                 /* NEGEDGE  */
    COND = 294,                    /* COND  */
    CONDELSE = 295,                /* CONDELSE  */
    QSTRING = 296,                 /* QSTRING  */
    ID = 297,                      /* ID  */
    FNUMBER = 298,                 /* FNUMBER  */
    DNUMBER = 299,                 /* DNUMBER  */
    EXPR_OPEN_IOPATH = 300,        /* EXPR_OPEN_IOPATH  */
    EXPR_OPEN = 301,               /* EXPR_OPEN  */
    EXPR_ID_CLOSE = 302            /* EXPR_ID_CLOSE  */
  };
  typedef enum yytokentype yytoken_kind_t;
#endif

/* Value type.  */
#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED
union YYSTYPE
{
#line 35 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"

  char character;
  const char *string;
  float number;
  float *number_ptr;
  int integer;
  sta::SdfTriple *triple;
  sta::SdfTripleSeq *delval_list;
  sta::SdfPortSpec *port_spec;
  sta::Transition *transition;

#line 123 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.hh"

};
typedef union YYSTYPE YYSTYPE;
# define YYSTYPE_IS_TRIVIAL 1
# define YYSTYPE_IS_DECLARED 1
#endif


extern YYSTYPE SdfParse_lval;


int SdfParse_parse (void);


#endif /* !YY_SDFPARSE_HOME_CAPTAINNOTHING_OPENROAD_BUILD_SRC_STA_SDFPARSE_HH_INCLUDED  */
