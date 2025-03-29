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

#ifndef YY_VERILOGPARSE_HOME_CAPTAINNOTHING_OPENROAD_BUILD_SRC_STA_VERILOGPARSE_HH_INCLUDED
# define YY_VERILOGPARSE_HOME_CAPTAINNOTHING_OPENROAD_BUILD_SRC_STA_VERILOGPARSE_HH_INCLUDED
/* Debug traces.  */
#ifndef YYDEBUG
# define YYDEBUG 0
#endif
#if YYDEBUG
extern int VerilogParse_debug;
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
    INT = 258,                     /* INT  */
    CONSTANT = 259,                /* CONSTANT  */
    ID = 260,                      /* ID  */
    STRING = 261,                  /* STRING  */
    MODULE = 262,                  /* MODULE  */
    ENDMODULE = 263,               /* ENDMODULE  */
    ASSIGN = 264,                  /* ASSIGN  */
    PARAMETER = 265,               /* PARAMETER  */
    DEFPARAM = 266,                /* DEFPARAM  */
    WIRE = 267,                    /* WIRE  */
    WAND = 268,                    /* WAND  */
    WOR = 269,                     /* WOR  */
    TRI = 270,                     /* TRI  */
    INPUT = 271,                   /* INPUT  */
    OUTPUT = 272,                  /* OUTPUT  */
    INOUT = 273,                   /* INOUT  */
    SUPPLY1 = 274,                 /* SUPPLY1  */
    SUPPLY0 = 275,                 /* SUPPLY0  */
    REG = 276,                     /* REG  */
    ATTRIBUTE_OPEN = 277,          /* ATTRIBUTE_OPEN  */
    ATTRIBUTE_CLOSED = 278,        /* ATTRIBUTE_CLOSED  */
    NEG = 279                      /* NEG  */
  };
  typedef enum yytokentype yytoken_kind_t;
#endif

/* Value type.  */
#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED
union YYSTYPE
{
#line 34 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"

  int ival;
  const char *string;
  const char *constant;
  const char *attribute_spec_value;
  sta::VerilogModule *module;
  sta::VerilogStmt *stmt;
  sta::VerilogStmtSeq *stmt_seq;
  sta::PortDirection *port_type;
  sta::VerilogDclArgSeq *dcl_arg_seq;
  sta::VerilogDclArg *dcl_arg;
  sta::VerilogAssign *assign;
  sta::VerilogInst *inst;
  sta::VerilogNet *net;
  sta::VerilogNetBitSelect *net_bit;
  sta::VerilogNetSeq *nets;
  sta::VerilogAttributeEntry *attribute_entry;
  sta::VerilogAttributeEntrySeq *attribute_seq;
  sta::VerilogAttributeStmt *attribute_stmt;
  sta::VerilogAttributeStmtSeq *attribute_stmt_seq;

#line 110 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.hh"

};
typedef union YYSTYPE YYSTYPE;
# define YYSTYPE_IS_TRIVIAL 1
# define YYSTYPE_IS_DECLARED 1
#endif


extern YYSTYPE VerilogParse_lval;


int VerilogParse_parse (void);


#endif /* !YY_VERILOGPARSE_HOME_CAPTAINNOTHING_OPENROAD_BUILD_SRC_STA_VERILOGPARSE_HH_INCLUDED  */
