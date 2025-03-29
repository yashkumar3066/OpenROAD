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

#ifndef YY_SPEFPARSE_HOME_CAPTAINNOTHING_OPENROAD_BUILD_SRC_STA_SPEFPARSE_HH_INCLUDED
# define YY_SPEFPARSE_HOME_CAPTAINNOTHING_OPENROAD_BUILD_SRC_STA_SPEFPARSE_HH_INCLUDED
/* Debug traces.  */
#ifndef YYDEBUG
# define YYDEBUG 0
#endif
#if YYDEBUG
extern int SpefParse_debug;
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
    SPEF = 258,                    /* SPEF  */
    DESIGN = 259,                  /* DESIGN  */
    DATE = 260,                    /* DATE  */
    VENDOR = 261,                  /* VENDOR  */
    PROGRAM = 262,                 /* PROGRAM  */
    DESIGN_FLOW = 263,             /* DESIGN_FLOW  */
    PVERSION = 264,                /* PVERSION  */
    DIVIDER = 265,                 /* DIVIDER  */
    DELIMITER = 266,               /* DELIMITER  */
    BUS_DELIMITER = 267,           /* BUS_DELIMITER  */
    T_UNIT = 268,                  /* T_UNIT  */
    C_UNIT = 269,                  /* C_UNIT  */
    R_UNIT = 270,                  /* R_UNIT  */
    L_UNIT = 271,                  /* L_UNIT  */
    NAME_MAP = 272,                /* NAME_MAP  */
    POWER_NETS = 273,              /* POWER_NETS  */
    GROUND_NETS = 274,             /* GROUND_NETS  */
    KW_C = 275,                    /* KW_C  */
    KW_L = 276,                    /* KW_L  */
    KW_S = 277,                    /* KW_S  */
    KW_D = 278,                    /* KW_D  */
    KW_V = 279,                    /* KW_V  */
    PORTS = 280,                   /* PORTS  */
    PHYSICAL_PORTS = 281,          /* PHYSICAL_PORTS  */
    DEFINE = 282,                  /* DEFINE  */
    PDEFINE = 283,                 /* PDEFINE  */
    D_NET = 284,                   /* D_NET  */
    D_PNET = 285,                  /* D_PNET  */
    R_NET = 286,                   /* R_NET  */
    R_PNET = 287,                  /* R_PNET  */
    END = 288,                     /* END  */
    CONN = 289,                    /* CONN  */
    CAP = 290,                     /* CAP  */
    RES = 291,                     /* RES  */
    INDUC = 292,                   /* INDUC  */
    KW_P = 293,                    /* KW_P  */
    KW_I = 294,                    /* KW_I  */
    KW_N = 295,                    /* KW_N  */
    DRIVER = 296,                  /* DRIVER  */
    CELL = 297,                    /* CELL  */
    C2_R1_C1 = 298,                /* C2_R1_C1  */
    LOADS = 299,                   /* LOADS  */
    RC = 300,                      /* RC  */
    KW_Q = 301,                    /* KW_Q  */
    KW_K = 302,                    /* KW_K  */
    INTEGER = 303,                 /* INTEGER  */
    FLOAT = 304,                   /* FLOAT  */
    QSTRING = 305,                 /* QSTRING  */
    INDEX = 306,                   /* INDEX  */
    IDENT = 307,                   /* IDENT  */
    NAME = 308                     /* NAME  */
  };
  typedef enum yytokentype yytoken_kind_t;
#endif

/* Value type.  */
#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED
union YYSTYPE
{
#line 32 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"

  char ch;
  char *string;
  int integer;
  float number;
  sta::StringSeq *string_seq;
  sta::PortDirection *port_dir;
  sta::SpefRspfPi *pi;
  sta::SpefTriple *triple;
  sta::Pin *pin;
  sta::Net *net;

#line 130 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.hh"

};
typedef union YYSTYPE YYSTYPE;
# define YYSTYPE_IS_TRIVIAL 1
# define YYSTYPE_IS_DECLARED 1
#endif


extern YYSTYPE SpefParse_lval;


int SpefParse_parse (void);


#endif /* !YY_SPEFPARSE_HOME_CAPTAINNOTHING_OPENROAD_BUILD_SRC_STA_SPEFPARSE_HH_INCLUDED  */
