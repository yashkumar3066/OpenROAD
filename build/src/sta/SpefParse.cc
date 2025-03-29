/* A Bison parser, made by GNU Bison 3.8.2.  */

/* Bison implementation for Yacc-like parsers in C

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

/* C LALR(1) parser skeleton written by Richard Stallman, by
   simplifying the original so-called "semantic" parser.  */

/* DO NOT RELY ON FEATURES THAT ARE NOT DOCUMENTED in the manual,
   especially those whose name start with YY_ or yy_.  They are
   private implementation details that can be changed or removed.  */

/* All symbols defined below should begin with yy or YY, to avoid
   infringing on user name space.  This should be done even for local
   variables, as they might otherwise be expanded by user macros.
   There are some unavoidable exceptions within include files to
   define necessary library symbols; they are noted "INFRINGES ON
   USER NAME SPACE" below.  */

/* Identify Bison output, and Bison version.  */
#define YYBISON 30802

/* Bison version string.  */
#define YYBISON_VERSION "3.8.2"

/* Skeleton name.  */
#define YYSKELETON_NAME "yacc.c"

/* Pure parsers.  */
#define YYPURE 0

/* Push parsers.  */
#define YYPUSH 0

/* Pull parsers.  */
#define YYPULL 1


/* Substitute the variable and function names.  */
#define yyparse         SpefParse_parse
#define yylex           SpefParse_lex
#define yyerror         SpefParse_error
#define yydebug         SpefParse_debug
#define yynerrs         SpefParse_nerrs
#define yylval          SpefParse_lval
#define yychar          SpefParse_char

/* First part of user prologue.  */
#line 1 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"


// OpenSTA, Static Timing Analyzer
// Copyright (c) 2024, Parallax Software, Inc.
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

#include <cstring>

#include "StringUtil.hh"
#include "StringSeq.hh"
#include "parasitics/SpefReaderPvt.hh"

int SpefLex_lex();
#define SpefParse_lex SpefLex_lex
// use yacc generated parser errors
#define YYERROR_VERBOSE


#line 109 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"

# ifndef YY_CAST
#  ifdef __cplusplus
#   define YY_CAST(Type, Val) static_cast<Type> (Val)
#   define YY_REINTERPRET_CAST(Type, Val) reinterpret_cast<Type> (Val)
#  else
#   define YY_CAST(Type, Val) ((Type) (Val))
#   define YY_REINTERPRET_CAST(Type, Val) ((Type) (Val))
#  endif
# endif
# ifndef YY_NULLPTR
#  if defined __cplusplus
#   if 201103L <= __cplusplus
#    define YY_NULLPTR nullptr
#   else
#    define YY_NULLPTR 0
#   endif
#  else
#   define YY_NULLPTR ((void*)0)
#  endif
# endif

#include "SpefParse.hh"
/* Symbol kind.  */
enum yysymbol_kind_t
{
  YYSYMBOL_YYEMPTY = -2,
  YYSYMBOL_YYEOF = 0,                      /* "end of file"  */
  YYSYMBOL_YYerror = 1,                    /* error  */
  YYSYMBOL_YYUNDEF = 2,                    /* "invalid token"  */
  YYSYMBOL_SPEF = 3,                       /* SPEF  */
  YYSYMBOL_DESIGN = 4,                     /* DESIGN  */
  YYSYMBOL_DATE = 5,                       /* DATE  */
  YYSYMBOL_VENDOR = 6,                     /* VENDOR  */
  YYSYMBOL_PROGRAM = 7,                    /* PROGRAM  */
  YYSYMBOL_DESIGN_FLOW = 8,                /* DESIGN_FLOW  */
  YYSYMBOL_PVERSION = 9,                   /* PVERSION  */
  YYSYMBOL_DIVIDER = 10,                   /* DIVIDER  */
  YYSYMBOL_DELIMITER = 11,                 /* DELIMITER  */
  YYSYMBOL_BUS_DELIMITER = 12,             /* BUS_DELIMITER  */
  YYSYMBOL_T_UNIT = 13,                    /* T_UNIT  */
  YYSYMBOL_C_UNIT = 14,                    /* C_UNIT  */
  YYSYMBOL_R_UNIT = 15,                    /* R_UNIT  */
  YYSYMBOL_L_UNIT = 16,                    /* L_UNIT  */
  YYSYMBOL_NAME_MAP = 17,                  /* NAME_MAP  */
  YYSYMBOL_POWER_NETS = 18,                /* POWER_NETS  */
  YYSYMBOL_GROUND_NETS = 19,               /* GROUND_NETS  */
  YYSYMBOL_KW_C = 20,                      /* KW_C  */
  YYSYMBOL_KW_L = 21,                      /* KW_L  */
  YYSYMBOL_KW_S = 22,                      /* KW_S  */
  YYSYMBOL_KW_D = 23,                      /* KW_D  */
  YYSYMBOL_KW_V = 24,                      /* KW_V  */
  YYSYMBOL_PORTS = 25,                     /* PORTS  */
  YYSYMBOL_PHYSICAL_PORTS = 26,            /* PHYSICAL_PORTS  */
  YYSYMBOL_DEFINE = 27,                    /* DEFINE  */
  YYSYMBOL_PDEFINE = 28,                   /* PDEFINE  */
  YYSYMBOL_D_NET = 29,                     /* D_NET  */
  YYSYMBOL_D_PNET = 30,                    /* D_PNET  */
  YYSYMBOL_R_NET = 31,                     /* R_NET  */
  YYSYMBOL_R_PNET = 32,                    /* R_PNET  */
  YYSYMBOL_END = 33,                       /* END  */
  YYSYMBOL_CONN = 34,                      /* CONN  */
  YYSYMBOL_CAP = 35,                       /* CAP  */
  YYSYMBOL_RES = 36,                       /* RES  */
  YYSYMBOL_INDUC = 37,                     /* INDUC  */
  YYSYMBOL_KW_P = 38,                      /* KW_P  */
  YYSYMBOL_KW_I = 39,                      /* KW_I  */
  YYSYMBOL_KW_N = 40,                      /* KW_N  */
  YYSYMBOL_DRIVER = 41,                    /* DRIVER  */
  YYSYMBOL_CELL = 42,                      /* CELL  */
  YYSYMBOL_C2_R1_C1 = 43,                  /* C2_R1_C1  */
  YYSYMBOL_LOADS = 44,                     /* LOADS  */
  YYSYMBOL_RC = 45,                        /* RC  */
  YYSYMBOL_KW_Q = 46,                      /* KW_Q  */
  YYSYMBOL_KW_K = 47,                      /* KW_K  */
  YYSYMBOL_INTEGER = 48,                   /* INTEGER  */
  YYSYMBOL_FLOAT = 49,                     /* FLOAT  */
  YYSYMBOL_QSTRING = 50,                   /* QSTRING  */
  YYSYMBOL_INDEX = 51,                     /* INDEX  */
  YYSYMBOL_IDENT = 52,                     /* IDENT  */
  YYSYMBOL_NAME = 53,                      /* NAME  */
  YYSYMBOL_54_ = 54,                       /* '['  */
  YYSYMBOL_55_ = 55,                       /* '{'  */
  YYSYMBOL_56_ = 56,                       /* '('  */
  YYSYMBOL_57_ = 57,                       /* '<'  */
  YYSYMBOL_58_ = 58,                       /* ']'  */
  YYSYMBOL_59_ = 59,                       /* '}'  */
  YYSYMBOL_60_ = 60,                       /* ')'  */
  YYSYMBOL_61_ = 61,                       /* '>'  */
  YYSYMBOL_62_ = 62,                       /* '.'  */
  YYSYMBOL_63_ = 63,                       /* '/'  */
  YYSYMBOL_64_ = 64,                       /* '|'  */
  YYSYMBOL_65_ = 65,                       /* ':'  */
  YYSYMBOL_YYACCEPT = 66,                  /* $accept  */
  YYSYMBOL_file = 67,                      /* file  */
  YYSYMBOL_prefix_bus_delim = 68,          /* prefix_bus_delim  */
  YYSYMBOL_suffix_bus_delim = 69,          /* suffix_bus_delim  */
  YYSYMBOL_hchar = 70,                     /* hchar  */
  YYSYMBOL_header_def = 71,                /* header_def  */
  YYSYMBOL_spef_version = 72,              /* spef_version  */
  YYSYMBOL_design_name = 73,               /* design_name  */
  YYSYMBOL_date = 74,                      /* date  */
  YYSYMBOL_program_name = 75,              /* program_name  */
  YYSYMBOL_program_version = 76,           /* program_version  */
  YYSYMBOL_vendor = 77,                    /* vendor  */
  YYSYMBOL_design_flow = 78,               /* design_flow  */
  YYSYMBOL_qstrings = 79,                  /* qstrings  */
  YYSYMBOL_hierarchy_div_def = 80,         /* hierarchy_div_def  */
  YYSYMBOL_pin_delim_def = 81,             /* pin_delim_def  */
  YYSYMBOL_bus_delim_def = 82,             /* bus_delim_def  */
  YYSYMBOL_unit_def = 83,                  /* unit_def  */
  YYSYMBOL_time_scale = 84,                /* time_scale  */
  YYSYMBOL_cap_scale = 85,                 /* cap_scale  */
  YYSYMBOL_res_scale = 86,                 /* res_scale  */
  YYSYMBOL_induc_scale = 87,               /* induc_scale  */
  YYSYMBOL_name_map = 88,                  /* name_map  */
  YYSYMBOL_name_map_entries = 89,          /* name_map_entries  */
  YYSYMBOL_name_map_entry = 90,            /* name_map_entry  */
  YYSYMBOL_mapped_item = 91,               /* mapped_item  */
  YYSYMBOL_power_def = 92,                 /* power_def  */
  YYSYMBOL_power_net_def = 93,             /* power_net_def  */
  YYSYMBOL_ground_net_def = 94,            /* ground_net_def  */
  YYSYMBOL_net_names = 95,                 /* net_names  */
  YYSYMBOL_net_name = 96,                  /* net_name  */
  YYSYMBOL_external_def = 97,              /* external_def  */
  YYSYMBOL_port_def = 98,                  /* port_def  */
  YYSYMBOL_port_entries = 99,              /* port_entries  */
  YYSYMBOL_port_entry = 100,               /* port_entry  */
  YYSYMBOL_direction = 101,                /* direction  */
  YYSYMBOL_port_name = 102,                /* port_name  */
  YYSYMBOL_inst_name = 103,                /* inst_name  */
  YYSYMBOL_physical_port_def = 104,        /* physical_port_def  */
  YYSYMBOL_pport_entries = 105,            /* pport_entries  */
  YYSYMBOL_pport_entry = 106,              /* pport_entry  */
  YYSYMBOL_pport_name = 107,               /* pport_name  */
  YYSYMBOL_pport = 108,                    /* pport  */
  YYSYMBOL_physical_inst = 109,            /* physical_inst  */
  YYSYMBOL_conn_attrs = 110,               /* conn_attrs  */
  YYSYMBOL_conn_attr = 111,                /* conn_attr  */
  YYSYMBOL_coordinates = 112,              /* coordinates  */
  YYSYMBOL_cap_load = 113,                 /* cap_load  */
  YYSYMBOL_par_value = 114,                /* par_value  */
  YYSYMBOL_slews = 115,                    /* slews  */
  YYSYMBOL_threshold = 116,                /* threshold  */
  YYSYMBOL_driving_cell = 117,             /* driving_cell  */
  YYSYMBOL_cell_type = 118,                /* cell_type  */
  YYSYMBOL_define_def = 119,               /* define_def  */
  YYSYMBOL_define_entry = 120,             /* define_entry  */
  YYSYMBOL_entity = 121,                   /* entity  */
  YYSYMBOL_internal_def = 122,             /* internal_def  */
  YYSYMBOL_nets = 123,                     /* nets  */
  YYSYMBOL_d_net = 124,                    /* d_net  */
  YYSYMBOL_125_1 = 125,                    /* $@1  */
  YYSYMBOL_net = 126,                      /* net  */
  YYSYMBOL_total_cap = 127,                /* total_cap  */
  YYSYMBOL_routing_conf = 128,             /* routing_conf  */
  YYSYMBOL_conf = 129,                     /* conf  */
  YYSYMBOL_conn_sec = 130,                 /* conn_sec  */
  YYSYMBOL_conn_defs = 131,                /* conn_defs  */
  YYSYMBOL_conn_def = 132,                 /* conn_def  */
  YYSYMBOL_external_connection = 133,      /* external_connection  */
  YYSYMBOL_internal_connection = 134,      /* internal_connection  */
  YYSYMBOL_pin_name = 135,                 /* pin_name  */
  YYSYMBOL_internal_node_coords = 136,     /* internal_node_coords  */
  YYSYMBOL_internal_node_coord = 137,      /* internal_node_coord  */
  YYSYMBOL_internal_parasitic_node = 138,  /* internal_parasitic_node  */
  YYSYMBOL_cap_sec = 139,                  /* cap_sec  */
  YYSYMBOL_cap_elems = 140,                /* cap_elems  */
  YYSYMBOL_cap_elem = 141,                 /* cap_elem  */
  YYSYMBOL_cap_id = 142,                   /* cap_id  */
  YYSYMBOL_parasitic_node = 143,           /* parasitic_node  */
  YYSYMBOL_res_sec = 144,                  /* res_sec  */
  YYSYMBOL_res_elems = 145,                /* res_elems  */
  YYSYMBOL_res_elem = 146,                 /* res_elem  */
  YYSYMBOL_res_id = 147,                   /* res_id  */
  YYSYMBOL_induc_sec = 148,                /* induc_sec  */
  YYSYMBOL_induc_elems = 149,              /* induc_elems  */
  YYSYMBOL_induc_elem = 150,               /* induc_elem  */
  YYSYMBOL_induc_id = 151,                 /* induc_id  */
  YYSYMBOL_r_net = 152,                    /* r_net  */
  YYSYMBOL_153_2 = 153,                    /* $@2  */
  YYSYMBOL_driver_reducs = 154,            /* driver_reducs  */
  YYSYMBOL_driver_reduc = 155,             /* driver_reduc  */
  YYSYMBOL_156_3 = 156,                    /* $@3  */
  YYSYMBOL_driver_pair = 157,              /* driver_pair  */
  YYSYMBOL_driver_cell = 158,              /* driver_cell  */
  YYSYMBOL_pi_model = 159,                 /* pi_model  */
  YYSYMBOL_load_desc = 160,                /* load_desc  */
  YYSYMBOL_rc_descs = 161,                 /* rc_descs  */
  YYSYMBOL_rc_desc = 162,                  /* rc_desc  */
  YYSYMBOL_pole_residue_desc = 163,        /* pole_residue_desc  */
  YYSYMBOL_pole_desc = 164,                /* pole_desc  */
  YYSYMBOL_poles = 165,                    /* poles  */
  YYSYMBOL_pole = 166,                     /* pole  */
  YYSYMBOL_complex_par_value = 167,        /* complex_par_value  */
  YYSYMBOL_cnumber = 168,                  /* cnumber  */
  YYSYMBOL_real_component = 169,           /* real_component  */
  YYSYMBOL_imaginary_component = 170,      /* imaginary_component  */
  YYSYMBOL_residue_desc = 171,             /* residue_desc  */
  YYSYMBOL_residues = 172,                 /* residues  */
  YYSYMBOL_residue = 173,                  /* residue  */
  YYSYMBOL_d_pnet = 174,                   /* d_pnet  */
  YYSYMBOL_pnet_ref = 175,                 /* pnet_ref  */
  YYSYMBOL_pconn_sec = 176,                /* pconn_sec  */
  YYSYMBOL_pconn_defs = 177,               /* pconn_defs  */
  YYSYMBOL_pconn_def = 178,                /* pconn_def  */
  YYSYMBOL_pexternal_connection = 179,     /* pexternal_connection  */
  YYSYMBOL_internal_pnode_coords = 180,    /* internal_pnode_coords  */
  YYSYMBOL_internal_pnode_coord = 181,     /* internal_pnode_coord  */
  YYSYMBOL_internal_pdspf_node = 182,      /* internal_pdspf_node  */
  YYSYMBOL_name_or_index = 183,            /* name_or_index  */
  YYSYMBOL_r_pnet = 184,                   /* r_pnet  */
  YYSYMBOL_pdriver_reduc = 185,            /* pdriver_reduc  */
  YYSYMBOL_pdriver_pair = 186,             /* pdriver_pair  */
  YYSYMBOL_number = 187,                   /* number  */
  YYSYMBOL_pos_integer = 188,              /* pos_integer  */
  YYSYMBOL_pos_number = 189                /* pos_number  */
};
typedef enum yysymbol_kind_t yysymbol_kind_t;


/* Second part of user prologue.  */
#line 93 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"


#line 335 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"


#ifdef short
# undef short
#endif

/* On compilers that do not define __PTRDIFF_MAX__ etc., make sure
   <limits.h> and (if available) <stdint.h> are included
   so that the code can choose integer types of a good width.  */

#ifndef __PTRDIFF_MAX__
# include <limits.h> /* INFRINGES ON USER NAME SPACE */
# if defined __STDC_VERSION__ && 199901 <= __STDC_VERSION__
#  include <stdint.h> /* INFRINGES ON USER NAME SPACE */
#  define YY_STDINT_H
# endif
#endif

/* Narrow types that promote to a signed type and that can represent a
   signed or unsigned integer of at least N bits.  In tables they can
   save space and decrease cache pressure.  Promoting to a signed type
   helps avoid bugs in integer arithmetic.  */

#ifdef __INT_LEAST8_MAX__
typedef __INT_LEAST8_TYPE__ yytype_int8;
#elif defined YY_STDINT_H
typedef int_least8_t yytype_int8;
#else
typedef signed char yytype_int8;
#endif

#ifdef __INT_LEAST16_MAX__
typedef __INT_LEAST16_TYPE__ yytype_int16;
#elif defined YY_STDINT_H
typedef int_least16_t yytype_int16;
#else
typedef short yytype_int16;
#endif

/* Work around bug in HP-UX 11.23, which defines these macros
   incorrectly for preprocessor constants.  This workaround can likely
   be removed in 2023, as HPE has promised support for HP-UX 11.23
   (aka HP-UX 11i v2) only through the end of 2022; see Table 2 of
   <https://h20195.www2.hpe.com/V2/getpdf.aspx/4AA4-7673ENW.pdf>.  */
#ifdef __hpux
# undef UINT_LEAST8_MAX
# undef UINT_LEAST16_MAX
# define UINT_LEAST8_MAX 255
# define UINT_LEAST16_MAX 65535
#endif

#if defined __UINT_LEAST8_MAX__ && __UINT_LEAST8_MAX__ <= __INT_MAX__
typedef __UINT_LEAST8_TYPE__ yytype_uint8;
#elif (!defined __UINT_LEAST8_MAX__ && defined YY_STDINT_H \
       && UINT_LEAST8_MAX <= INT_MAX)
typedef uint_least8_t yytype_uint8;
#elif !defined __UINT_LEAST8_MAX__ && UCHAR_MAX <= INT_MAX
typedef unsigned char yytype_uint8;
#else
typedef short yytype_uint8;
#endif

#if defined __UINT_LEAST16_MAX__ && __UINT_LEAST16_MAX__ <= __INT_MAX__
typedef __UINT_LEAST16_TYPE__ yytype_uint16;
#elif (!defined __UINT_LEAST16_MAX__ && defined YY_STDINT_H \
       && UINT_LEAST16_MAX <= INT_MAX)
typedef uint_least16_t yytype_uint16;
#elif !defined __UINT_LEAST16_MAX__ && USHRT_MAX <= INT_MAX
typedef unsigned short yytype_uint16;
#else
typedef int yytype_uint16;
#endif

#ifndef YYPTRDIFF_T
# if defined __PTRDIFF_TYPE__ && defined __PTRDIFF_MAX__
#  define YYPTRDIFF_T __PTRDIFF_TYPE__
#  define YYPTRDIFF_MAXIMUM __PTRDIFF_MAX__
# elif defined PTRDIFF_MAX
#  ifndef ptrdiff_t
#   include <stddef.h> /* INFRINGES ON USER NAME SPACE */
#  endif
#  define YYPTRDIFF_T ptrdiff_t
#  define YYPTRDIFF_MAXIMUM PTRDIFF_MAX
# else
#  define YYPTRDIFF_T long
#  define YYPTRDIFF_MAXIMUM LONG_MAX
# endif
#endif

#ifndef YYSIZE_T
# ifdef __SIZE_TYPE__
#  define YYSIZE_T __SIZE_TYPE__
# elif defined size_t
#  define YYSIZE_T size_t
# elif defined __STDC_VERSION__ && 199901 <= __STDC_VERSION__
#  include <stddef.h> /* INFRINGES ON USER NAME SPACE */
#  define YYSIZE_T size_t
# else
#  define YYSIZE_T unsigned
# endif
#endif

#define YYSIZE_MAXIMUM                                  \
  YY_CAST (YYPTRDIFF_T,                                 \
           (YYPTRDIFF_MAXIMUM < YY_CAST (YYSIZE_T, -1)  \
            ? YYPTRDIFF_MAXIMUM                         \
            : YY_CAST (YYSIZE_T, -1)))

#define YYSIZEOF(X) YY_CAST (YYPTRDIFF_T, sizeof (X))


/* Stored state numbers (used for stacks). */
typedef yytype_int16 yy_state_t;

/* State numbers in computations.  */
typedef int yy_state_fast_t;

#ifndef YY_
# if defined YYENABLE_NLS && YYENABLE_NLS
#  if ENABLE_NLS
#   include <libintl.h> /* INFRINGES ON USER NAME SPACE */
#   define YY_(Msgid) dgettext ("bison-runtime", Msgid)
#  endif
# endif
# ifndef YY_
#  define YY_(Msgid) Msgid
# endif
#endif


#ifndef YY_ATTRIBUTE_PURE
# if defined __GNUC__ && 2 < __GNUC__ + (96 <= __GNUC_MINOR__)
#  define YY_ATTRIBUTE_PURE __attribute__ ((__pure__))
# else
#  define YY_ATTRIBUTE_PURE
# endif
#endif

#ifndef YY_ATTRIBUTE_UNUSED
# if defined __GNUC__ && 2 < __GNUC__ + (7 <= __GNUC_MINOR__)
#  define YY_ATTRIBUTE_UNUSED __attribute__ ((__unused__))
# else
#  define YY_ATTRIBUTE_UNUSED
# endif
#endif

/* Suppress unused-variable warnings by "using" E.  */
#if ! defined lint || defined __GNUC__
# define YY_USE(E) ((void) (E))
#else
# define YY_USE(E) /* empty */
#endif

/* Suppress an incorrect diagnostic about yylval being uninitialized.  */
#if defined __GNUC__ && ! defined __ICC && 406 <= __GNUC__ * 100 + __GNUC_MINOR__
# if __GNUC__ * 100 + __GNUC_MINOR__ < 407
#  define YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN                           \
    _Pragma ("GCC diagnostic push")                                     \
    _Pragma ("GCC diagnostic ignored \"-Wuninitialized\"")
# else
#  define YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN                           \
    _Pragma ("GCC diagnostic push")                                     \
    _Pragma ("GCC diagnostic ignored \"-Wuninitialized\"")              \
    _Pragma ("GCC diagnostic ignored \"-Wmaybe-uninitialized\"")
# endif
# define YY_IGNORE_MAYBE_UNINITIALIZED_END      \
    _Pragma ("GCC diagnostic pop")
#else
# define YY_INITIAL_VALUE(Value) Value
#endif
#ifndef YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
# define YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
# define YY_IGNORE_MAYBE_UNINITIALIZED_END
#endif
#ifndef YY_INITIAL_VALUE
# define YY_INITIAL_VALUE(Value) /* Nothing. */
#endif

#if defined __cplusplus && defined __GNUC__ && ! defined __ICC && 6 <= __GNUC__
# define YY_IGNORE_USELESS_CAST_BEGIN                          \
    _Pragma ("GCC diagnostic push")                            \
    _Pragma ("GCC diagnostic ignored \"-Wuseless-cast\"")
# define YY_IGNORE_USELESS_CAST_END            \
    _Pragma ("GCC diagnostic pop")
#endif
#ifndef YY_IGNORE_USELESS_CAST_BEGIN
# define YY_IGNORE_USELESS_CAST_BEGIN
# define YY_IGNORE_USELESS_CAST_END
#endif


#define YY_ASSERT(E) ((void) (0 && (E)))

#if !defined yyoverflow

/* The parser invokes alloca or malloc; define the necessary symbols.  */

# ifdef YYSTACK_USE_ALLOCA
#  if YYSTACK_USE_ALLOCA
#   ifdef __GNUC__
#    define YYSTACK_ALLOC __builtin_alloca
#   elif defined __BUILTIN_VA_ARG_INCR
#    include <alloca.h> /* INFRINGES ON USER NAME SPACE */
#   elif defined _AIX
#    define YYSTACK_ALLOC __alloca
#   elif defined _MSC_VER
#    include <malloc.h> /* INFRINGES ON USER NAME SPACE */
#    define alloca _alloca
#   else
#    define YYSTACK_ALLOC alloca
#    if ! defined _ALLOCA_H && ! defined EXIT_SUCCESS
#     include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
      /* Use EXIT_SUCCESS as a witness for stdlib.h.  */
#     ifndef EXIT_SUCCESS
#      define EXIT_SUCCESS 0
#     endif
#    endif
#   endif
#  endif
# endif

# ifdef YYSTACK_ALLOC
   /* Pacify GCC's 'empty if-body' warning.  */
#  define YYSTACK_FREE(Ptr) do { /* empty */; } while (0)
#  ifndef YYSTACK_ALLOC_MAXIMUM
    /* The OS might guarantee only one guard page at the bottom of the stack,
       and a page size can be as small as 4096 bytes.  So we cannot safely
       invoke alloca (N) if N exceeds 4096.  Use a slightly smaller number
       to allow for a few compiler-allocated temporary stack slots.  */
#   define YYSTACK_ALLOC_MAXIMUM 4032 /* reasonable circa 2006 */
#  endif
# else
#  define YYSTACK_ALLOC YYMALLOC
#  define YYSTACK_FREE YYFREE
#  ifndef YYSTACK_ALLOC_MAXIMUM
#   define YYSTACK_ALLOC_MAXIMUM YYSIZE_MAXIMUM
#  endif
#  if (defined __cplusplus && ! defined EXIT_SUCCESS \
       && ! ((defined YYMALLOC || defined malloc) \
             && (defined YYFREE || defined free)))
#   include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#   ifndef EXIT_SUCCESS
#    define EXIT_SUCCESS 0
#   endif
#  endif
#  ifndef YYMALLOC
#   define YYMALLOC malloc
#   if ! defined malloc && ! defined EXIT_SUCCESS
void *malloc (YYSIZE_T); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
#  ifndef YYFREE
#   define YYFREE free
#   if ! defined free && ! defined EXIT_SUCCESS
void free (void *); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
# endif
#endif /* !defined yyoverflow */

#if (! defined yyoverflow \
     && (! defined __cplusplus \
         || (defined YYSTYPE_IS_TRIVIAL && YYSTYPE_IS_TRIVIAL)))

/* A type that is properly aligned for any stack member.  */
union yyalloc
{
  yy_state_t yyss_alloc;
  YYSTYPE yyvs_alloc;
};

/* The size of the maximum gap between one aligned stack and the next.  */
# define YYSTACK_GAP_MAXIMUM (YYSIZEOF (union yyalloc) - 1)

/* The size of an array large to enough to hold all stacks, each with
   N elements.  */
# define YYSTACK_BYTES(N) \
     ((N) * (YYSIZEOF (yy_state_t) + YYSIZEOF (YYSTYPE)) \
      + YYSTACK_GAP_MAXIMUM)

# define YYCOPY_NEEDED 1

/* Relocate STACK from its old location to the new one.  The
   local variables YYSIZE and YYSTACKSIZE give the old and new number of
   elements in the stack, and YYPTR gives the new location of the
   stack.  Advance YYPTR to a properly aligned location for the next
   stack.  */
# define YYSTACK_RELOCATE(Stack_alloc, Stack)                           \
    do                                                                  \
      {                                                                 \
        YYPTRDIFF_T yynewbytes;                                         \
        YYCOPY (&yyptr->Stack_alloc, Stack, yysize);                    \
        Stack = &yyptr->Stack_alloc;                                    \
        yynewbytes = yystacksize * YYSIZEOF (*Stack) + YYSTACK_GAP_MAXIMUM; \
        yyptr += yynewbytes / YYSIZEOF (*yyptr);                        \
      }                                                                 \
    while (0)

#endif

#if defined YYCOPY_NEEDED && YYCOPY_NEEDED
/* Copy COUNT objects from SRC to DST.  The source and destination do
   not overlap.  */
# ifndef YYCOPY
#  if defined __GNUC__ && 1 < __GNUC__
#   define YYCOPY(Dst, Src, Count) \
      __builtin_memcpy (Dst, Src, YY_CAST (YYSIZE_T, (Count)) * sizeof (*(Src)))
#  else
#   define YYCOPY(Dst, Src, Count)              \
      do                                        \
        {                                       \
          YYPTRDIFF_T yyi;                      \
          for (yyi = 0; yyi < (Count); yyi++)   \
            (Dst)[yyi] = (Src)[yyi];            \
        }                                       \
      while (0)
#  endif
# endif
#endif /* !YYCOPY_NEEDED */

/* YYFINAL -- State number of the termination state.  */
#define YYFINAL  6
/* YYLAST -- Last index in YYTABLE.  */
#define YYLAST   253

/* YYNTOKENS -- Number of terminals.  */
#define YYNTOKENS  66
/* YYNNTS -- Number of nonterminals.  */
#define YYNNTS  124
/* YYNRULES -- Number of rules.  */
#define YYNRULES  193
/* YYNSTATES -- Number of states.  */
#define YYNSTATES  335

/* YYMAXUTOK -- Last valid token kind.  */
#define YYMAXUTOK   308


/* YYTRANSLATE(TOKEN-NUM) -- Symbol number corresponding to TOKEN-NUM
   as returned by yylex, with out-of-bounds checking.  */
#define YYTRANSLATE(YYX)                                \
  (0 <= (YYX) && (YYX) <= YYMAXUTOK                     \
   ? YY_CAST (yysymbol_kind_t, yytranslate[YYX])        \
   : YYSYMBOL_YYUNDEF)

/* YYTRANSLATE[TOKEN-NUM] -- Symbol number corresponding to TOKEN-NUM
   as returned by yylex.  */
static const yytype_int8 yytranslate[] =
{
       0,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
      56,    60,     2,     2,     2,     2,    62,    63,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,    65,     2,
      57,     2,    61,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,    54,     2,    58,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,    55,    64,    59,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     1,     2,     3,     4,
       5,     6,     7,     8,     9,    10,    11,    12,    13,    14,
      15,    16,    17,    18,    19,    20,    21,    22,    23,    24,
      25,    26,    27,    28,    29,    30,    31,    32,    33,    34,
      35,    36,    37,    38,    39,    40,    41,    42,    43,    44,
      45,    46,    47,    48,    49,    50,    51,    52,    53
};

#if YYDEBUG
/* YYRLINE[YYN] -- Source line where rule number YYN was defined.  */
static const yytype_int16 yyrline[] =
{
       0,    99,    99,   110,   112,   114,   116,   121,   123,   125,
     127,   132,   134,   136,   138,   145,   159,   164,   169,   174,
     179,   184,   189,   194,   198,   203,   208,   213,   215,   222,
     229,   234,   239,   244,   250,   252,   256,   257,   261,   268,
     269,   270,   275,   277,   278,   279,   283,   287,   291,   292,
     296,   302,   304,   305,   306,   310,   314,   315,   319,   324,
     331,   335,   339,   343,   344,   348,   352,   354,   361,   365,
     370,   372,   376,   377,   378,   379,   383,   387,   392,   394,
     399,   403,   410,   411,   415,   420,   421,   426,   428,   432,
     436,   441,   448,   454,   455,   459,   460,   461,   462,   469,
     468,   475,   482,   485,   487,   491,   496,   498,   502,   503,
     507,   508,   512,   514,   521,   525,   531,   533,   537,   541,
     547,   549,   554,   555,   559,   561,   566,   570,   575,   577,
     582,   583,   587,   592,   597,   599,   604,   605,   609,   614,
     621,   620,   626,   628,   633,   632,   641,   646,   651,   658,
     662,   663,   667,   669,   674,   678,   682,   683,   687,   691,
     692,   693,   694,   698,   703,   707,   711,   715,   716,   720,
     726,   732,   736,   740,   741,   745,   746,   750,   753,   755,
     759,   763,   771,   772,   773,   779,   781,   786,   790,   796,
     798,   802,   811,   817
};
#endif

/** Accessing symbol of state STATE.  */
#define YY_ACCESSING_SYMBOL(State) YY_CAST (yysymbol_kind_t, yystos[State])

#if YYDEBUG || 0
/* The user-facing name of the symbol whose (internal) number is
   YYSYMBOL.  No bounds checking.  */
static const char *yysymbol_name (yysymbol_kind_t yysymbol) YY_ATTRIBUTE_UNUSED;

/* YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
   First, the terminals, then, starting at YYNTOKENS, nonterminals.  */
static const char *const yytname[] =
{
  "\"end of file\"", "error", "\"invalid token\"", "SPEF", "DESIGN",
  "DATE", "VENDOR", "PROGRAM", "DESIGN_FLOW", "PVERSION", "DIVIDER",
  "DELIMITER", "BUS_DELIMITER", "T_UNIT", "C_UNIT", "R_UNIT", "L_UNIT",
  "NAME_MAP", "POWER_NETS", "GROUND_NETS", "KW_C", "KW_L", "KW_S", "KW_D",
  "KW_V", "PORTS", "PHYSICAL_PORTS", "DEFINE", "PDEFINE", "D_NET",
  "D_PNET", "R_NET", "R_PNET", "END", "CONN", "CAP", "RES", "INDUC",
  "KW_P", "KW_I", "KW_N", "DRIVER", "CELL", "C2_R1_C1", "LOADS", "RC",
  "KW_Q", "KW_K", "INTEGER", "FLOAT", "QSTRING", "INDEX", "IDENT", "NAME",
  "'['", "'{'", "'('", "'<'", "']'", "'}'", "')'", "'>'", "'.'", "'/'",
  "'|'", "':'", "$accept", "file", "prefix_bus_delim", "suffix_bus_delim",
  "hchar", "header_def", "spef_version", "design_name", "date",
  "program_name", "program_version", "vendor", "design_flow", "qstrings",
  "hierarchy_div_def", "pin_delim_def", "bus_delim_def", "unit_def",
  "time_scale", "cap_scale", "res_scale", "induc_scale", "name_map",
  "name_map_entries", "name_map_entry", "mapped_item", "power_def",
  "power_net_def", "ground_net_def", "net_names", "net_name",
  "external_def", "port_def", "port_entries", "port_entry", "direction",
  "port_name", "inst_name", "physical_port_def", "pport_entries",
  "pport_entry", "pport_name", "pport", "physical_inst", "conn_attrs",
  "conn_attr", "coordinates", "cap_load", "par_value", "slews",
  "threshold", "driving_cell", "cell_type", "define_def", "define_entry",
  "entity", "internal_def", "nets", "d_net", "$@1", "net", "total_cap",
  "routing_conf", "conf", "conn_sec", "conn_defs", "conn_def",
  "external_connection", "internal_connection", "pin_name",
  "internal_node_coords", "internal_node_coord", "internal_parasitic_node",
  "cap_sec", "cap_elems", "cap_elem", "cap_id", "parasitic_node",
  "res_sec", "res_elems", "res_elem", "res_id", "induc_sec", "induc_elems",
  "induc_elem", "induc_id", "r_net", "$@2", "driver_reducs",
  "driver_reduc", "$@3", "driver_pair", "driver_cell", "pi_model",
  "load_desc", "rc_descs", "rc_desc", "pole_residue_desc", "pole_desc",
  "poles", "pole", "complex_par_value", "cnumber", "real_component",
  "imaginary_component", "residue_desc", "residues", "residue", "d_pnet",
  "pnet_ref", "pconn_sec", "pconn_defs", "pconn_def",
  "pexternal_connection", "internal_pnode_coords", "internal_pnode_coord",
  "internal_pdspf_node", "name_or_index", "r_pnet", "pdriver_reduc",
  "pdriver_pair", "number", "pos_integer", "pos_number", YY_NULLPTR
};

static const char *
yysymbol_name (yysymbol_kind_t yysymbol)
{
  return yytname[yysymbol];
}
#endif

#define YYPACT_NINF (-307)

#define yypact_value_is_default(Yyn) \
  ((Yyn) == YYPACT_NINF)

#define YYTABLE_NINF (-113)

#define yytable_value_is_error(Yyn) \
  0

/* YYPACT[STATE-NUM] -- Index in YYTABLE of the portion describing
   STATE-NUM.  */
static const yytype_int16 yypact[] =
{
      44,   -12,    66,    56,    80,  -307,  -307,    63,   105,    70,
     124,    65,    63,  -307,    45,    45,    35,   120,  -307,  -307,
      90,   136,  -307,  -307,  -307,  -307,  -307,  -307,  -307,  -307,
      45,  -307,  -307,    45,    45,    45,  -307,   117,  -307,  -307,
    -307,    94,   138,  -307,    45,  -307,    96,  -307,    45,  -307,
      98,    82,    84,    49,  -307,  -307,   101,   143,  -307,  -307,
    -307,  -307,  -307,    45,    45,    45,    45,    45,    45,    45,
    -307,    12,  -307,  -307,  -307,  -307,  -307,  -307,   103,   148,
      68,    68,  -307,  -307,    42,  -307,   107,  -307,    77,  -307,
      77,  -307,    77,    77,  -307,  -307,   108,   149,    77,    77,
      77,    76,  -307,  -307,  -307,  -307,  -307,  -307,   107,  -307,
    -307,  -307,  -307,  -307,  -307,   100,   137,  -307,   137,  -307,
     110,    38,   151,    77,  -307,    77,  -307,  -307,  -307,  -307,
     137,    77,   115,   132,   137,   -23,  -307,  -307,  -307,  -307,
    -307,  -307,    38,   155,  -307,    83,   134,   104,  -307,  -307,
    -307,    95,   135,  -307,  -307,    45,   140,   129,  -307,    50,
     161,  -307,  -307,    83,   111,    97,   135,    77,    45,    45,
      95,  -307,  -307,   139,    26,  -307,  -307,  -307,  -307,    76,
     141,  -307,  -307,  -307,  -307,    51,    83,  -307,   165,  -307,
      83,    45,    45,    97,  -307,   139,  -307,  -307,    96,    96,
    -307,   142,   115,  -307,   144,  -307,    45,  -307,   129,  -307,
      77,   150,  -307,  -307,  -307,  -307,  -307,   128,    83,   168,
     121,   122,    96,   133,    96,  -307,   153,   144,    68,    68,
      45,  -307,  -307,    45,  -307,   115,  -307,   162,  -307,   141,
      77,   152,  -307,  -307,   146,    83,   172,    83,    45,  -307,
    -307,    45,  -307,   166,  -307,  -307,   180,  -307,    34,  -307,
    -307,    45,  -307,   115,  -307,  -307,    77,    45,   152,  -307,
    -307,   154,    83,  -307,  -307,  -307,    68,    68,   180,  -307,
    -307,  -307,  -307,    77,    45,  -307,    45,  -307,   150,  -307,
      77,  -307,  -307,   157,  -307,  -307,    77,    45,  -307,   158,
    -307,  -307,    77,   115,  -307,   156,  -307,     9,   115,  -307,
      77,     9,  -307,  -307,   145,   147,     9,    77,  -307,  -307,
     163,    77,  -307,     9,  -307,   164,  -307,   167,   170,  -307,
    -307,   163,    77,  -307,  -307
};

/* YYDEFACT[STATE-NUM] -- Default reduction number in state STATE-NUM.
   Performed when YYTABLE does not specify something else to do.  Zero
   means the default is an error.  */
static const yytype_uint8 yydefact[] =
{
       0,     0,     0,    34,     0,    16,     1,     0,    42,     0,
       0,     0,    35,    36,     0,     0,    51,    43,    44,    17,
       0,     0,    41,    39,    40,    38,    37,   184,   182,   183,
      46,    48,    50,    47,     0,     0,    87,    52,    53,    45,
      18,     0,     0,    49,    55,    56,     0,    60,    62,    63,
       0,     0,    66,     0,    54,    21,     0,     0,    57,    59,
      70,    64,    70,     0,     0,     0,     0,     0,     0,     0,
      88,     2,    93,    95,    96,    97,    98,    19,     0,     0,
      58,    65,    67,    68,     0,    61,     0,    69,     0,   101,
       0,   171,     0,     0,    94,    20,     0,     0,     0,     0,
       0,     0,    71,    72,    73,    74,    75,    92,     0,    89,
      91,   189,   190,   102,    99,    78,   103,   140,   103,    23,
      22,     0,     0,     0,    77,     0,    86,    85,    84,    90,
     103,     0,     0,     0,   103,     0,    24,    11,    12,    13,
      14,    25,     0,     0,    76,    80,   106,     0,   191,   104,
     105,     0,   120,   142,   185,     0,     0,     0,    26,     0,
       0,   192,   193,     0,    82,     0,   120,     0,     0,     0,
     178,   173,   122,   128,     0,   188,   114,   115,   186,     0,
       0,     3,     4,     5,     6,    27,     0,    15,     0,    81,
       0,     0,     0,   116,   108,   128,    79,   177,     0,     0,
     174,   172,   121,   130,   134,   141,     0,   143,     0,   147,
       0,     0,     7,     8,     9,    10,    28,     0,     0,     0,
       0,     0,     0,    69,     0,   109,   107,   134,     0,     0,
       0,   179,   123,     0,   126,   129,   136,     0,   146,     0,
       0,     0,   187,    30,     0,     0,     0,     0,     0,    70,
      70,     0,   117,     0,   175,   176,     0,   181,     0,   127,
     131,     0,   133,   135,   170,   144,     0,     0,   149,   150,
      31,     0,     0,    29,    83,   113,   110,   111,     0,   119,
     100,   180,   124,     0,     0,   137,     0,   139,     0,   148,
       0,   151,    32,     0,   118,   125,     0,     0,   145,   152,
      33,   132,     0,     0,   153,     0,   138,     0,     0,   154,
       0,   155,   156,   158,   159,   160,     0,     0,   164,   157,
       0,     0,   169,   166,   167,     0,   165,     0,     0,   168,
     163,     0,     0,   161,   162
};

/* YYPGOTO[NTERM-NUM].  */
static const yytype_int16 yypgoto[] =
{
    -307,  -307,  -307,  -307,    59,  -307,  -307,  -307,  -307,  -307,
    -307,  -307,  -307,  -307,  -307,  -307,  -307,  -307,  -307,  -307,
    -307,  -307,  -307,  -307,   193,  -307,  -307,  -307,   190,   198,
      -4,  -307,  -307,  -307,   173,  -176,  -307,   159,   181,  -307,
     174,    55,   -22,   -61,   -59,   -91,  -245,  -307,   -94,  -307,
      67,  -307,    52,  -307,  -307,   -71,  -307,   169,  -307,  -307,
     160,    29,  -106,  -307,  -307,  -307,    36,  -307,  -160,  -198,
    -307,  -307,  -307,    72,  -307,  -307,  -307,  -222,    46,  -307,
    -307,  -307,     6,  -307,  -307,  -307,  -307,  -307,  -307,  -307,
    -307,  -307,    31,     3,   -44,  -307,   -21,  -307,  -307,  -307,
     -66,  -260,  -306,  -307,  -307,  -307,  -307,   -77,  -307,   179,
    -307,  -307,    79,  -307,  -307,  -307,  -307,   -14,  -307,  -307,
    -307,   -96,  -195,  -173
};

/* YYDEFGOTO[NTERM-NUM].  */
static const yytype_int16 yydefgoto[] =
{
       0,     2,   185,   216,   141,     3,     4,    10,    21,    57,
      79,    42,    97,   120,   122,   143,   160,   187,   188,   219,
     246,   273,     8,    12,    13,    25,    16,    17,    18,    30,
      31,    36,    37,    44,    45,    60,    46,    84,    38,    48,
      49,    50,    82,    51,    80,   102,   103,   104,   113,   105,
     163,   106,   128,    53,    70,   109,    71,    72,    73,   130,
      88,   114,   133,   149,   166,   193,   194,   222,   175,   176,
     226,   252,   278,   173,   202,   232,   233,   258,   204,   235,
     260,   261,   237,   263,   285,   286,    74,   134,   174,   207,
     288,   208,   180,   211,   242,   268,   269,   304,   305,   311,
     312,   313,   314,   317,   325,   309,   323,   324,    75,    90,
     152,   170,   171,   198,   201,   231,   256,   259,    76,   156,
     157,   115,   150,   164
};

/* YYTABLE[YYPACT[STATE-NUM]] -- What to do in state STATE-NUM.  If
   positive, shift that token.  If negative, reduce the rule whose
   number is the opposite.  If YYTABLE_NINF, syntax error.  */
static const yytype_int16 yytable[] =
{
      32,    32,   123,    81,    86,   124,   125,   234,   238,   199,
     154,   281,   135,   217,   327,   110,    32,   220,   155,    32,
      47,    52,   228,   229,   146,   333,    43,   144,   153,    43,
      47,   145,   224,   294,    52,   147,   283,   129,     5,   284,
     262,    66,    67,    68,    69,   244,   249,     1,   250,    83,
      85,    87,    89,    91,    89,    91,   322,   111,   112,   205,
      34,    35,   296,   322,   297,   310,     6,   206,   287,   290,
      85,   196,   271,     7,   274,   302,    64,    65,    66,    67,
      68,    69,   111,   112,     9,    27,    28,    29,    98,    99,
     100,   101,   107,    27,    28,    29,    27,    28,    29,   293,
     137,   138,   139,   140,   181,   182,   183,   184,   307,   212,
     213,   214,   215,   316,    11,    22,   240,    23,    24,   116,
      19,   117,   118,    14,    15,   111,   112,   126,   127,    20,
     221,   161,   162,   168,   169,   191,   192,   254,   255,    15,
      40,   177,    41,    35,    55,    56,   266,    63,    59,   -69,
      62,    77,    78,    95,    52,   177,    96,   107,   119,   121,
     136,   132,   142,   148,   282,   131,   151,   159,   165,   167,
     172,   179,   289,   178,   186,   203,   190,   223,   177,   218,
     243,   236,   230,   245,   210,  -112,   247,   248,   272,   295,
     276,   277,   177,   251,   241,   264,   299,   267,   270,   280,
      98,   158,   301,   308,   303,    26,   292,    39,   306,   300,
     320,   315,   321,    33,   318,   315,   257,    58,    54,   310,
     315,   326,    61,   197,   330,   328,   275,   315,    92,   225,
     189,   209,   331,   253,    83,   332,   334,   279,   195,   239,
      94,   227,   265,   108,   298,   319,   329,   291,    93,   200,
       0,     0,     0,   177
};

static const yytype_int16 yycheck[] =
{
      14,    15,    98,    62,    65,    99,   100,   202,   206,   169,
      33,   256,   118,   186,   320,    86,    30,   190,    41,    33,
      34,    35,   198,   199,   130,   331,    30,   123,   134,    33,
      44,   125,   192,   278,    48,   131,   258,   108,    50,   261,
     235,    29,    30,    31,    32,   218,   222,     3,   224,    63,
      64,    65,    66,    67,    68,    69,   316,    48,    49,    33,
      25,    26,   284,   323,   286,    56,     0,    41,   263,   267,
      84,   167,   245,    17,   247,   297,    27,    28,    29,    30,
      31,    32,    48,    49,     4,    51,    52,    53,    20,    21,
      22,    23,    50,    51,    52,    53,    51,    52,    53,   272,
      62,    63,    64,    65,    54,    55,    56,    57,   303,    58,
      59,    60,    61,   308,    51,    50,   210,    52,    53,    90,
      50,    92,    93,    18,    19,    48,    49,    51,    52,     5,
     191,    48,    49,    38,    39,    38,    39,   228,   229,    19,
      50,   155,     6,    26,    50,     7,   240,    65,    52,    65,
      52,    50,     9,    50,   168,   169,     8,    50,    50,    10,
      50,    24,    11,    48,   258,    65,    34,    12,    34,    65,
      35,    42,   266,    33,    13,    36,    65,   191,   192,    14,
      52,    37,    40,    15,    43,    52,    65,    65,    16,   283,
     249,   250,   206,    40,    44,    33,   290,    45,    52,    33,
      20,   142,   296,    47,    46,    12,    52,    17,   302,    52,
      65,   307,    65,    15,   310,   311,   230,    44,    37,    56,
     316,   317,    48,   168,    60,   321,   248,   323,    68,   193,
     163,   179,    65,   227,   248,    65,   332,   251,   166,   208,
      71,   195,   239,    84,   288,   311,   323,   268,    69,   170,
      -1,    -1,    -1,   267
};

/* YYSTOS[STATE-NUM] -- The symbol kind of the accessing symbol of
   state STATE-NUM.  */
static const yytype_uint8 yystos[] =
{
       0,     3,    67,    71,    72,    50,     0,    17,    88,     4,
      73,    51,    89,    90,    18,    19,    92,    93,    94,    50,
       5,    74,    50,    52,    53,    91,    90,    51,    52,    53,
      95,    96,   183,    95,    25,    26,    97,    98,   104,    94,
      50,     6,    77,    96,    99,   100,   102,   183,   105,   106,
     107,   109,   183,   119,   104,    50,     7,    75,   100,    52,
     101,   106,    52,    65,    27,    28,    29,    30,    31,    32,
     120,   122,   123,   124,   152,   174,   184,    50,     9,    76,
     110,   110,   108,   183,   103,   183,   109,   183,   126,   183,
     175,   183,   126,   175,   123,    50,     8,    78,    20,    21,
      22,    23,   111,   112,   113,   115,   117,    50,   103,   121,
     121,    48,    49,   114,   127,   187,   127,   127,   127,    50,
      79,    10,    80,   187,   114,   114,    51,    52,   118,   121,
     125,    65,    24,   128,   153,   128,    50,    62,    63,    64,
      65,    70,    11,    81,   187,   114,   128,   187,    48,   129,
     188,    34,   176,   128,    33,    41,   185,   186,    70,    12,
      82,    48,    49,   116,   189,    34,   130,    65,    38,    39,
     177,   178,    35,   139,   154,   134,   135,   183,    33,    42,
     158,    54,    55,    56,    57,    68,    13,    83,    84,   116,
      65,    38,    39,   131,   132,   139,   187,   107,   179,   134,
     178,   180,   140,    36,   144,    33,    41,   155,   157,   118,
      43,   159,    58,    59,    60,    61,    69,   189,    14,    85,
     189,   109,   133,   183,   134,   132,   136,   144,   101,   101,
      40,   181,   141,   142,   188,   145,    37,   148,   135,   158,
     114,    44,   160,    52,   189,    15,    86,    65,    65,   101,
     101,    40,   137,   148,   111,   111,   182,   183,   143,   183,
     146,   147,   188,   149,    33,   159,   114,    45,   161,   162,
      52,   189,    16,    87,   189,   108,   110,   110,   138,   183,
      33,   112,   114,   143,   143,   150,   151,   188,   156,   114,
     135,   162,    52,   189,   112,   114,   143,   143,   160,   114,
      52,   114,   143,    46,   163,   164,   114,   188,    47,   171,
      56,   165,   166,   167,   168,   187,   188,   169,   187,   166,
      65,    65,   167,   172,   173,   170,   187,   168,   187,   173,
      60,    65,    65,   168,   187
};

/* YYR1[RULE-NUM] -- Symbol kind of the left-hand side of rule RULE-NUM.  */
static const yytype_uint8 yyr1[] =
{
       0,    66,    67,    68,    68,    68,    68,    69,    69,    69,
      69,    70,    70,    70,    70,    71,    72,    73,    74,    75,
      76,    77,    78,    79,    79,    80,    81,    82,    82,    83,
      84,    85,    86,    87,    88,    88,    89,    89,    90,    91,
      91,    91,    92,    92,    92,    92,    93,    94,    95,    95,
      96,    97,    97,    97,    97,    98,    99,    99,   100,   101,
     102,   103,   104,   105,   105,   106,   107,   107,   108,   109,
     110,   110,   111,   111,   111,   111,   112,   113,   114,   114,
     115,   115,   116,   116,   117,   118,   118,   119,   119,   120,
     120,   120,   121,   122,   122,   123,   123,   123,   123,   125,
     124,   126,   127,   128,   128,   129,   130,   130,   131,   131,
     132,   132,   133,   133,   134,   135,   136,   136,   137,   138,
     139,   139,   140,   140,   141,   141,   142,   143,   144,   144,
     145,   145,   146,   147,   148,   148,   149,   149,   150,   151,
     153,   152,   154,   154,   156,   155,   157,   158,   159,   160,
     161,   161,   162,   162,   163,   164,   165,   165,   166,   167,
     167,   167,   167,   168,   169,   170,   171,   172,   172,   173,
     174,   175,   176,   177,   177,   178,   178,   179,   180,   180,
     181,   182,   183,   183,   183,   184,   184,   185,   186,   187,
     187,   188,   189,   189
};

/* YYR2[RULE-NUM] -- Number of symbols on the right-hand side of rule RULE-NUM.  */
static const yytype_int8 yyr2[] =
{
       0,     2,     6,     1,     1,     1,     1,     1,     1,     1,
       1,     1,     1,     1,     1,    11,     2,     2,     2,     2,
       2,     2,     2,     1,     2,     2,     2,     2,     3,     4,
       3,     3,     3,     3,     0,     2,     1,     2,     2,     1,
       1,     1,     0,     1,     1,     2,     2,     2,     1,     2,
       1,     0,     1,     1,     2,     2,     1,     2,     3,     1,
       1,     1,     2,     1,     2,     3,     1,     3,     1,     1,
       0,     2,     1,     1,     1,     1,     3,     2,     1,     5,
       3,     5,     1,     5,     2,     1,     1,     0,     2,     3,
       4,     3,     1,     1,     2,     1,     1,     1,     1,     0,
      10,     1,     1,     0,     2,     1,     0,     3,     1,     2,
       4,     4,     1,     3,     1,     1,     0,     2,     3,     1,
       0,     2,     0,     2,     3,     4,     1,     1,     0,     2,
       0,     2,     4,     1,     0,     2,     0,     2,     4,     1,
       0,     7,     0,     2,     0,     5,     2,     2,     4,     2,
       1,     2,     3,     4,     2,     3,     1,     2,     1,     1,
       1,     5,     5,     4,     1,     1,     3,     1,     2,     1,
       9,     1,     3,     1,     2,     4,     4,     1,     0,     2,
       3,     1,     1,     1,     1,     5,     6,     4,     2,     1,
       1,     1,     1,     1
};


enum { YYENOMEM = -2 };

#define yyerrok         (yyerrstatus = 0)
#define yyclearin       (yychar = YYEMPTY)

#define YYACCEPT        goto yyacceptlab
#define YYABORT         goto yyabortlab
#define YYERROR         goto yyerrorlab
#define YYNOMEM         goto yyexhaustedlab


#define YYRECOVERING()  (!!yyerrstatus)

#define YYBACKUP(Token, Value)                                    \
  do                                                              \
    if (yychar == YYEMPTY)                                        \
      {                                                           \
        yychar = (Token);                                         \
        yylval = (Value);                                         \
        YYPOPSTACK (yylen);                                       \
        yystate = *yyssp;                                         \
        goto yybackup;                                            \
      }                                                           \
    else                                                          \
      {                                                           \
        yyerror (YY_("syntax error: cannot back up")); \
        YYERROR;                                                  \
      }                                                           \
  while (0)

/* Backward compatibility with an undocumented macro.
   Use YYerror or YYUNDEF. */
#define YYERRCODE YYUNDEF


/* Enable debugging if requested.  */
#if YYDEBUG

# ifndef YYFPRINTF
#  include <stdio.h> /* INFRINGES ON USER NAME SPACE */
#  define YYFPRINTF fprintf
# endif

# define YYDPRINTF(Args)                        \
do {                                            \
  if (yydebug)                                  \
    YYFPRINTF Args;                             \
} while (0)




# define YY_SYMBOL_PRINT(Title, Kind, Value, Location)                    \
do {                                                                      \
  if (yydebug)                                                            \
    {                                                                     \
      YYFPRINTF (stderr, "%s ", Title);                                   \
      yy_symbol_print (stderr,                                            \
                  Kind, Value); \
      YYFPRINTF (stderr, "\n");                                           \
    }                                                                     \
} while (0)


/*-----------------------------------.
| Print this symbol's value on YYO.  |
`-----------------------------------*/

static void
yy_symbol_value_print (FILE *yyo,
                       yysymbol_kind_t yykind, YYSTYPE const * const yyvaluep)
{
  FILE *yyoutput = yyo;
  YY_USE (yyoutput);
  if (!yyvaluep)
    return;
  YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
  YY_USE (yykind);
  YY_IGNORE_MAYBE_UNINITIALIZED_END
}


/*---------------------------.
| Print this symbol on YYO.  |
`---------------------------*/

static void
yy_symbol_print (FILE *yyo,
                 yysymbol_kind_t yykind, YYSTYPE const * const yyvaluep)
{
  YYFPRINTF (yyo, "%s %s (",
             yykind < YYNTOKENS ? "token" : "nterm", yysymbol_name (yykind));

  yy_symbol_value_print (yyo, yykind, yyvaluep);
  YYFPRINTF (yyo, ")");
}

/*------------------------------------------------------------------.
| yy_stack_print -- Print the state stack from its BOTTOM up to its |
| TOP (included).                                                   |
`------------------------------------------------------------------*/

static void
yy_stack_print (yy_state_t *yybottom, yy_state_t *yytop)
{
  YYFPRINTF (stderr, "Stack now");
  for (; yybottom <= yytop; yybottom++)
    {
      int yybot = *yybottom;
      YYFPRINTF (stderr, " %d", yybot);
    }
  YYFPRINTF (stderr, "\n");
}

# define YY_STACK_PRINT(Bottom, Top)                            \
do {                                                            \
  if (yydebug)                                                  \
    yy_stack_print ((Bottom), (Top));                           \
} while (0)


/*------------------------------------------------.
| Report that the YYRULE is going to be reduced.  |
`------------------------------------------------*/

static void
yy_reduce_print (yy_state_t *yyssp, YYSTYPE *yyvsp,
                 int yyrule)
{
  int yylno = yyrline[yyrule];
  int yynrhs = yyr2[yyrule];
  int yyi;
  YYFPRINTF (stderr, "Reducing stack by rule %d (line %d):\n",
             yyrule - 1, yylno);
  /* The symbols being reduced.  */
  for (yyi = 0; yyi < yynrhs; yyi++)
    {
      YYFPRINTF (stderr, "   $%d = ", yyi + 1);
      yy_symbol_print (stderr,
                       YY_ACCESSING_SYMBOL (+yyssp[yyi + 1 - yynrhs]),
                       &yyvsp[(yyi + 1) - (yynrhs)]);
      YYFPRINTF (stderr, "\n");
    }
}

# define YY_REDUCE_PRINT(Rule)          \
do {                                    \
  if (yydebug)                          \
    yy_reduce_print (yyssp, yyvsp, Rule); \
} while (0)

/* Nonzero means print parse trace.  It is left uninitialized so that
   multiple parsers can coexist.  */
int yydebug;
#else /* !YYDEBUG */
# define YYDPRINTF(Args) ((void) 0)
# define YY_SYMBOL_PRINT(Title, Kind, Value, Location)
# define YY_STACK_PRINT(Bottom, Top)
# define YY_REDUCE_PRINT(Rule)
#endif /* !YYDEBUG */


/* YYINITDEPTH -- initial size of the parser's stacks.  */
#ifndef YYINITDEPTH
# define YYINITDEPTH 200
#endif

/* YYMAXDEPTH -- maximum size the stacks can grow to (effective only
   if the built-in stack extension method is used).

   Do not make this value too large; the results are undefined if
   YYSTACK_ALLOC_MAXIMUM < YYSTACK_BYTES (YYMAXDEPTH)
   evaluated with infinite-precision integer arithmetic.  */

#ifndef YYMAXDEPTH
# define YYMAXDEPTH 10000
#endif






/*-----------------------------------------------.
| Release the memory associated to this symbol.  |
`-----------------------------------------------*/

static void
yydestruct (const char *yymsg,
            yysymbol_kind_t yykind, YYSTYPE *yyvaluep)
{
  YY_USE (yyvaluep);
  if (!yymsg)
    yymsg = "Deleting";
  YY_SYMBOL_PRINT (yymsg, yykind, yyvaluep, yylocationp);

  YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
  YY_USE (yykind);
  YY_IGNORE_MAYBE_UNINITIALIZED_END
}


/* Lookahead token kind.  */
int yychar;

/* The semantic value of the lookahead symbol.  */
YYSTYPE yylval;
/* Number of syntax errors so far.  */
int yynerrs;




/*----------.
| yyparse.  |
`----------*/

int
yyparse (void)
{
    yy_state_fast_t yystate = 0;
    /* Number of tokens to shift before error messages enabled.  */
    int yyerrstatus = 0;

    /* Refer to the stacks through separate pointers, to allow yyoverflow
       to reallocate them elsewhere.  */

    /* Their size.  */
    YYPTRDIFF_T yystacksize = YYINITDEPTH;

    /* The state stack: array, bottom, top.  */
    yy_state_t yyssa[YYINITDEPTH];
    yy_state_t *yyss = yyssa;
    yy_state_t *yyssp = yyss;

    /* The semantic value stack: array, bottom, top.  */
    YYSTYPE yyvsa[YYINITDEPTH];
    YYSTYPE *yyvs = yyvsa;
    YYSTYPE *yyvsp = yyvs;

  int yyn;
  /* The return value of yyparse.  */
  int yyresult;
  /* Lookahead symbol kind.  */
  yysymbol_kind_t yytoken = YYSYMBOL_YYEMPTY;
  /* The variables used to return semantic value and location from the
     action routines.  */
  YYSTYPE yyval;



#define YYPOPSTACK(N)   (yyvsp -= (N), yyssp -= (N))

  /* The number of symbols on the RHS of the reduced rule.
     Keep to zero when no symbol should be popped.  */
  int yylen = 0;

  YYDPRINTF ((stderr, "Starting parse\n"));

  yychar = YYEMPTY; /* Cause a token to be read.  */

  goto yysetstate;


/*------------------------------------------------------------.
| yynewstate -- push a new state, which is found in yystate.  |
`------------------------------------------------------------*/
yynewstate:
  /* In all cases, when you get here, the value and location stacks
     have just been pushed.  So pushing a state here evens the stacks.  */
  yyssp++;


/*--------------------------------------------------------------------.
| yysetstate -- set current state (the top of the stack) to yystate.  |
`--------------------------------------------------------------------*/
yysetstate:
  YYDPRINTF ((stderr, "Entering state %d\n", yystate));
  YY_ASSERT (0 <= yystate && yystate < YYNSTATES);
  YY_IGNORE_USELESS_CAST_BEGIN
  *yyssp = YY_CAST (yy_state_t, yystate);
  YY_IGNORE_USELESS_CAST_END
  YY_STACK_PRINT (yyss, yyssp);

  if (yyss + yystacksize - 1 <= yyssp)
#if !defined yyoverflow && !defined YYSTACK_RELOCATE
    YYNOMEM;
#else
    {
      /* Get the current used size of the three stacks, in elements.  */
      YYPTRDIFF_T yysize = yyssp - yyss + 1;

# if defined yyoverflow
      {
        /* Give user a chance to reallocate the stack.  Use copies of
           these so that the &'s don't force the real ones into
           memory.  */
        yy_state_t *yyss1 = yyss;
        YYSTYPE *yyvs1 = yyvs;

        /* Each stack pointer address is followed by the size of the
           data in use in that stack, in bytes.  This used to be a
           conditional around just the two extra args, but that might
           be undefined if yyoverflow is a macro.  */
        yyoverflow (YY_("memory exhausted"),
                    &yyss1, yysize * YYSIZEOF (*yyssp),
                    &yyvs1, yysize * YYSIZEOF (*yyvsp),
                    &yystacksize);
        yyss = yyss1;
        yyvs = yyvs1;
      }
# else /* defined YYSTACK_RELOCATE */
      /* Extend the stack our own way.  */
      if (YYMAXDEPTH <= yystacksize)
        YYNOMEM;
      yystacksize *= 2;
      if (YYMAXDEPTH < yystacksize)
        yystacksize = YYMAXDEPTH;

      {
        yy_state_t *yyss1 = yyss;
        union yyalloc *yyptr =
          YY_CAST (union yyalloc *,
                   YYSTACK_ALLOC (YY_CAST (YYSIZE_T, YYSTACK_BYTES (yystacksize))));
        if (! yyptr)
          YYNOMEM;
        YYSTACK_RELOCATE (yyss_alloc, yyss);
        YYSTACK_RELOCATE (yyvs_alloc, yyvs);
#  undef YYSTACK_RELOCATE
        if (yyss1 != yyssa)
          YYSTACK_FREE (yyss1);
      }
# endif

      yyssp = yyss + yysize - 1;
      yyvsp = yyvs + yysize - 1;

      YY_IGNORE_USELESS_CAST_BEGIN
      YYDPRINTF ((stderr, "Stack size increased to %ld\n",
                  YY_CAST (long, yystacksize)));
      YY_IGNORE_USELESS_CAST_END

      if (yyss + yystacksize - 1 <= yyssp)
        YYABORT;
    }
#endif /* !defined yyoverflow && !defined YYSTACK_RELOCATE */


  if (yystate == YYFINAL)
    YYACCEPT;

  goto yybackup;


/*-----------.
| yybackup.  |
`-----------*/
yybackup:
  /* Do appropriate processing given the current state.  Read a
     lookahead token if we need one and don't already have one.  */

  /* First try to decide what to do without reference to lookahead token.  */
  yyn = yypact[yystate];
  if (yypact_value_is_default (yyn))
    goto yydefault;

  /* Not known => get a lookahead token if don't already have one.  */

  /* YYCHAR is either empty, or end-of-input, or a valid lookahead.  */
  if (yychar == YYEMPTY)
    {
      YYDPRINTF ((stderr, "Reading a token\n"));
      yychar = yylex ();
    }

  if (yychar <= YYEOF)
    {
      yychar = YYEOF;
      yytoken = YYSYMBOL_YYEOF;
      YYDPRINTF ((stderr, "Now at end of input.\n"));
    }
  else if (yychar == YYerror)
    {
      /* The scanner already issued an error message, process directly
         to error recovery.  But do not keep the error token as
         lookahead, it is too special and may lead us to an endless
         loop in error recovery. */
      yychar = YYUNDEF;
      yytoken = YYSYMBOL_YYerror;
      goto yyerrlab1;
    }
  else
    {
      yytoken = YYTRANSLATE (yychar);
      YY_SYMBOL_PRINT ("Next token is", yytoken, &yylval, &yylloc);
    }

  /* If the proper action on seeing token YYTOKEN is to reduce or to
     detect an error, take that action.  */
  yyn += yytoken;
  if (yyn < 0 || YYLAST < yyn || yycheck[yyn] != yytoken)
    goto yydefault;
  yyn = yytable[yyn];
  if (yyn <= 0)
    {
      if (yytable_value_is_error (yyn))
        goto yyerrlab;
      yyn = -yyn;
      goto yyreduce;
    }

  /* Count tokens shifted since error; after three, turn off error
     status.  */
  if (yyerrstatus)
    yyerrstatus--;

  /* Shift the lookahead token.  */
  YY_SYMBOL_PRINT ("Shifting", yytoken, &yylval, &yylloc);
  yystate = yyn;
  YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
  *++yyvsp = yylval;
  YY_IGNORE_MAYBE_UNINITIALIZED_END

  /* Discard the shifted token.  */
  yychar = YYEMPTY;
  goto yynewstate;


/*-----------------------------------------------------------.
| yydefault -- do the default action for the current state.  |
`-----------------------------------------------------------*/
yydefault:
  yyn = yydefact[yystate];
  if (yyn == 0)
    goto yyerrlab;
  goto yyreduce;


/*-----------------------------.
| yyreduce -- do a reduction.  |
`-----------------------------*/
yyreduce:
  /* yyn is the number of a rule to reduce with.  */
  yylen = yyr2[yyn];

  /* If YYLEN is nonzero, implement the default value of the action:
     '$$ = $1'.

     Otherwise, the following line sets YYVAL to garbage.
     This behavior is undocumented and Bison
     users should not rely upon it.  Assigning to YYVAL
     unconditionally makes the parser a bit smaller, and it avoids a
     GCC warning that YYVAL may be used uninitialized.  */
  yyval = yyvsp[1-yylen];


  YY_REDUCE_PRINT (yyn);
  switch (yyn)
    {
  case 3: /* prefix_bus_delim: '['  */
#line 111 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.ch) = '['; }
#line 1545 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 4: /* prefix_bus_delim: '{'  */
#line 113 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.ch) = '}'; }
#line 1551 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 5: /* prefix_bus_delim: '('  */
#line 115 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.ch) = ')'; }
#line 1557 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 6: /* prefix_bus_delim: '<'  */
#line 117 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.ch) = '<'; }
#line 1563 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 7: /* suffix_bus_delim: ']'  */
#line 122 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.ch) = ']'; }
#line 1569 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 8: /* suffix_bus_delim: '}'  */
#line 124 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.ch) = '}'; }
#line 1575 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 9: /* suffix_bus_delim: ')'  */
#line 126 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.ch) = ')'; }
#line 1581 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 10: /* suffix_bus_delim: '>'  */
#line 128 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.ch) = '>'; }
#line 1587 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 11: /* hchar: '.'  */
#line 133 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.ch) = '.'; }
#line 1593 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 12: /* hchar: '/'  */
#line 135 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.ch) = '/'; }
#line 1599 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 13: /* hchar: '|'  */
#line 137 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.ch) = '|'; }
#line 1605 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 14: /* hchar: ':'  */
#line 139 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.ch) = ':'; }
#line 1611 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 16: /* spef_version: SPEF QSTRING  */
#line 160 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::stringDelete((yyvsp[0].string)); }
#line 1617 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 17: /* design_name: DESIGN QSTRING  */
#line 165 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::stringDelete((yyvsp[0].string)); }
#line 1623 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 18: /* date: DATE QSTRING  */
#line 170 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::stringDelete((yyvsp[0].string)); }
#line 1629 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 19: /* program_name: PROGRAM QSTRING  */
#line 175 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::stringDelete((yyvsp[0].string)); }
#line 1635 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 20: /* program_version: PVERSION QSTRING  */
#line 180 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::stringDelete((yyvsp[0].string)); }
#line 1641 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 21: /* vendor: VENDOR QSTRING  */
#line 185 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::stringDelete((yyvsp[0].string)); }
#line 1647 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 22: /* design_flow: DESIGN_FLOW qstrings  */
#line 190 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::spef_reader->setDesignFlow((yyvsp[0].string_seq)); }
#line 1653 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 23: /* qstrings: QSTRING  */
#line 195 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.string_seq) = new sta::StringSeq;
	  (yyval.string_seq)->push_back((yyvsp[0].string));
	}
#line 1661 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 24: /* qstrings: qstrings QSTRING  */
#line 199 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.string_seq)->push_back((yyvsp[0].string)); }
#line 1667 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 25: /* hierarchy_div_def: DIVIDER hchar  */
#line 204 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::spef_reader->setDivider((yyvsp[0].ch)); }
#line 1673 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 26: /* pin_delim_def: DELIMITER hchar  */
#line 209 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::spef_reader->setDelimiter((yyvsp[0].ch)); }
#line 1679 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 27: /* bus_delim_def: BUS_DELIMITER prefix_bus_delim  */
#line 214 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::spef_reader->setBusBrackets((yyvsp[0].ch), '\0'); }
#line 1685 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 28: /* bus_delim_def: BUS_DELIMITER prefix_bus_delim suffix_bus_delim  */
#line 216 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::spef_reader->setBusBrackets((yyvsp[-1].ch), (yyvsp[0].ch)); }
#line 1691 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 30: /* time_scale: T_UNIT pos_number IDENT  */
#line 230 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::spef_reader->setTimeScale((yyvsp[-1].number), (yyvsp[0].string)); }
#line 1697 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 31: /* cap_scale: C_UNIT pos_number IDENT  */
#line 235 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::spef_reader->setCapScale((yyvsp[-1].number), (yyvsp[0].string)); }
#line 1703 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 32: /* res_scale: R_UNIT pos_number IDENT  */
#line 240 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::spef_reader->setResScale((yyvsp[-1].number), (yyvsp[0].string)); }
#line 1709 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 33: /* induc_scale: L_UNIT pos_number IDENT  */
#line 245 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::spef_reader->setInductScale((yyvsp[-1].number), (yyvsp[0].string)); }
#line 1715 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 38: /* name_map_entry: INDEX mapped_item  */
#line 262 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::spef_reader->makeNameMapEntry((yyvsp[-1].string), (yyvsp[0].string));
	  sta::stringDelete((yyvsp[-1].string));
	}
#line 1723 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 50: /* net_name: name_or_index  */
#line 297 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::stringDelete((yyvsp[0].string)); }
#line 1729 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 58: /* port_entry: port_name direction conn_attrs  */
#line 320 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::stringDelete((yyvsp[-2].string)); }
#line 1735 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 59: /* direction: IDENT  */
#line 325 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.port_dir) = sta::spef_reader->portDirection((yyvsp[0].string));
          sta::stringDelete((yyvsp[0].string));
	}
#line 1743 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 66: /* pport_name: name_or_index  */
#line 353 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::stringDelete((yyvsp[0].string)); }
#line 1749 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 67: /* pport_name: physical_inst ':' pport  */
#line 355 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::stringDelete((yyvsp[-2].string));
	  sta::stringDelete((yyvsp[0].string));
	}
#line 1757 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 77: /* cap_load: KW_L par_value  */
#line 388 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { delete (yyvsp[0].triple); }
#line 1763 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 78: /* par_value: number  */
#line 393 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.triple) = new sta::SpefTriple((yyvsp[0].number)); }
#line 1769 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 79: /* par_value: number ':' number ':' number  */
#line 395 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.triple) = new sta::SpefTriple((yyvsp[-4].number), (yyvsp[-2].number), (yyvsp[0].number)); }
#line 1775 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 80: /* slews: KW_S par_value par_value  */
#line 400 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { delete (yyvsp[-1].triple);
	  delete (yyvsp[0].triple);
	}
#line 1783 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 81: /* slews: KW_S par_value par_value threshold threshold  */
#line 404 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { delete (yyvsp[-3].triple);
	  delete (yyvsp[-2].triple);
	}
#line 1791 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 84: /* driving_cell: KW_D cell_type  */
#line 416 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::stringDelete((yyvsp[0].string)); }
#line 1797 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 89: /* define_entry: DEFINE inst_name entity  */
#line 433 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::stringDelete((yyvsp[-1].string));
	  sta::stringDelete((yyvsp[0].string));
	}
#line 1805 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 90: /* define_entry: DEFINE inst_name inst_name entity  */
#line 437 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::stringDelete((yyvsp[-2].string));
	  sta::stringDelete((yyvsp[-1].string));
	  sta::stringDelete((yyvsp[0].string));
	}
#line 1814 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 91: /* define_entry: PDEFINE physical_inst entity  */
#line 442 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::stringDelete((yyvsp[-1].string));
	  sta::stringDelete((yyvsp[0].string));
	}
#line 1822 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 99: /* $@1: %empty  */
#line 469 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::spef_reader->dspfBegin((yyvsp[-1].net), (yyvsp[0].triple)); }
#line 1828 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 100: /* d_net: D_NET net total_cap $@1 routing_conf conn_sec cap_sec res_sec induc_sec END  */
#line 471 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::spef_reader->dspfFinish(); }
#line 1834 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 101: /* net: name_or_index  */
#line 476 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.net) = sta::spef_reader->findNet((yyvsp[0].string));
	  sta::stringDelete((yyvsp[0].string));
	}
#line 1842 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 112: /* external_connection: name_or_index  */
#line 513 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::stringDelete((yyvsp[0].string)); }
#line 1848 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 113: /* external_connection: physical_inst ':' pport  */
#line 515 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::stringDelete((yyvsp[-2].string));
	  sta::stringDelete((yyvsp[0].string));
	}
#line 1856 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 115: /* pin_name: name_or_index  */
#line 526 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.pin) = sta::spef_reader->findPin((yyvsp[0].string));
	  sta::stringDelete((yyvsp[0].string));
	}
#line 1864 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 119: /* internal_parasitic_node: name_or_index  */
#line 542 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::stringDelete((yyvsp[0].string)); }
#line 1870 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 122: /* cap_elems: %empty  */
#line 554 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.integer) = 0; }
#line 1876 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 124: /* cap_elem: cap_id parasitic_node par_value  */
#line 560 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::spef_reader->makeCapacitor((yyvsp[-2].integer), (yyvsp[-1].string), (yyvsp[0].triple)); }
#line 1882 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 125: /* cap_elem: cap_id parasitic_node parasitic_node par_value  */
#line 562 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::spef_reader->makeCapacitor((yyvsp[-3].integer), (yyvsp[-2].string), (yyvsp[-1].string), (yyvsp[0].triple)); }
#line 1888 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 130: /* res_elems: %empty  */
#line 582 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.integer) = 0; }
#line 1894 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 132: /* res_elem: res_id parasitic_node parasitic_node par_value  */
#line 588 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::spef_reader->makeResistor((yyvsp[-3].integer), (yyvsp[-2].string), (yyvsp[-1].string), (yyvsp[0].triple)); }
#line 1900 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 136: /* induc_elems: %empty  */
#line 604 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.integer) = 0; }
#line 1906 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 138: /* induc_elem: induc_id parasitic_node parasitic_node par_value  */
#line 610 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { delete (yyvsp[0].triple); }
#line 1912 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 140: /* $@2: %empty  */
#line 621 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::spef_reader->rspfBegin((yyvsp[-1].net), (yyvsp[0].triple)); }
#line 1918 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 141: /* r_net: R_NET net total_cap $@2 routing_conf driver_reducs END  */
#line 623 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::spef_reader->rspfFinish(); }
#line 1924 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 144: /* $@3: %empty  */
#line 633 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::spef_reader->rspfDrvrBegin((yyvsp[-2].pin), (yyvsp[0].pi));
	  sta::stringDelete((yyvsp[-1].string));
	}
#line 1932 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 145: /* driver_reduc: driver_pair driver_cell pi_model $@3 load_desc  */
#line 637 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::spef_reader->rspfDrvrFinish(); }
#line 1938 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 146: /* driver_pair: DRIVER pin_name  */
#line 642 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.pin) = (yyvsp[0].pin); }
#line 1944 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 147: /* driver_cell: CELL cell_type  */
#line 647 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.string) = (yyvsp[0].string); }
#line 1950 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 148: /* pi_model: C2_R1_C1 par_value par_value par_value  */
#line 652 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.pi) = new sta::SpefRspfPi((yyvsp[-2].triple), (yyvsp[-1].triple), (yyvsp[0].triple)); }
#line 1956 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 152: /* rc_desc: RC pin_name par_value  */
#line 668 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::spef_reader->rspfLoad((yyvsp[-1].pin), (yyvsp[0].triple)); }
#line 1962 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 153: /* rc_desc: RC pin_name par_value pole_residue_desc  */
#line 670 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::spef_reader->rspfLoad((yyvsp[-2].pin), (yyvsp[-1].triple)); }
#line 1968 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 163: /* cnumber: '(' real_component imaginary_component ')'  */
#line 699 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.number) = (yyvsp[-2].number); }
#line 1974 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 170: /* d_pnet: D_PNET pnet_ref total_cap routing_conf pconn_sec cap_sec res_sec induc_sec END  */
#line 728 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::stringDelete((yyvsp[-7].string)); }
#line 1980 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 181: /* internal_pdspf_node: name_or_index  */
#line 764 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        {
	  sta::stringDelete((yyvsp[0].string));
	  (yyval.string) = 0;
	}
#line 1989 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 185: /* r_pnet: R_PNET pnet_ref total_cap routing_conf END  */
#line 780 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::stringDelete((yyvsp[-3].string)); }
#line 1995 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 186: /* r_pnet: R_PNET pnet_ref total_cap routing_conf pdriver_reduc END  */
#line 782 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { sta::stringDelete((yyvsp[-4].string)); }
#line 2001 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 189: /* number: INTEGER  */
#line 797 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { (yyval.number) = static_cast<float>((yyvsp[0].integer)); }
#line 2007 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 191: /* pos_integer: INTEGER  */
#line 803 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { int value = (yyvsp[0].integer);
	  if (value < 0)
	    sta::spef_reader->warn(1525, "%d is not positive.", value);
	  (yyval.integer) = value;
	}
#line 2017 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 192: /* pos_number: INTEGER  */
#line 812 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { float value = static_cast<float>((yyvsp[0].integer));
	  if (value < 0)
	    sta::spef_reader->warn(1526, "%.4f is not positive.", value);
	  (yyval.number) = value;
	}
#line 2027 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;

  case 193: /* pos_number: FLOAT  */
#line 818 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"
        { float value = static_cast<float>((yyvsp[0].number));
	  if (value < 0)
	    sta::spef_reader->warn(1527, "%.4f is not positive.", value);
	  (yyval.number) = value;
	}
#line 2037 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"
    break;


#line 2041 "/home/captainnothing/OpenROAD/build/src/sta/SpefParse.cc"

      default: break;
    }
  /* User semantic actions sometimes alter yychar, and that requires
     that yytoken be updated with the new translation.  We take the
     approach of translating immediately before every use of yytoken.
     One alternative is translating here after every semantic action,
     but that translation would be missed if the semantic action invokes
     YYABORT, YYACCEPT, or YYERROR immediately after altering yychar or
     if it invokes YYBACKUP.  In the case of YYABORT or YYACCEPT, an
     incorrect destructor might then be invoked immediately.  In the
     case of YYERROR or YYBACKUP, subsequent parser actions might lead
     to an incorrect destructor call or verbose syntax error message
     before the lookahead is translated.  */
  YY_SYMBOL_PRINT ("-> $$ =", YY_CAST (yysymbol_kind_t, yyr1[yyn]), &yyval, &yyloc);

  YYPOPSTACK (yylen);
  yylen = 0;

  *++yyvsp = yyval;

  /* Now 'shift' the result of the reduction.  Determine what state
     that goes to, based on the state we popped back to and the rule
     number reduced by.  */
  {
    const int yylhs = yyr1[yyn] - YYNTOKENS;
    const int yyi = yypgoto[yylhs] + *yyssp;
    yystate = (0 <= yyi && yyi <= YYLAST && yycheck[yyi] == *yyssp
               ? yytable[yyi]
               : yydefgoto[yylhs]);
  }

  goto yynewstate;


/*--------------------------------------.
| yyerrlab -- here on detecting error.  |
`--------------------------------------*/
yyerrlab:
  /* Make sure we have latest lookahead translation.  See comments at
     user semantic actions for why this is necessary.  */
  yytoken = yychar == YYEMPTY ? YYSYMBOL_YYEMPTY : YYTRANSLATE (yychar);
  /* If not already recovering from an error, report this error.  */
  if (!yyerrstatus)
    {
      ++yynerrs;
      yyerror (YY_("syntax error"));
    }

  if (yyerrstatus == 3)
    {
      /* If just tried and failed to reuse lookahead token after an
         error, discard it.  */

      if (yychar <= YYEOF)
        {
          /* Return failure if at end of input.  */
          if (yychar == YYEOF)
            YYABORT;
        }
      else
        {
          yydestruct ("Error: discarding",
                      yytoken, &yylval);
          yychar = YYEMPTY;
        }
    }

  /* Else will try to reuse lookahead token after shifting the error
     token.  */
  goto yyerrlab1;


/*---------------------------------------------------.
| yyerrorlab -- error raised explicitly by YYERROR.  |
`---------------------------------------------------*/
yyerrorlab:
  /* Pacify compilers when the user code never invokes YYERROR and the
     label yyerrorlab therefore never appears in user code.  */
  if (0)
    YYERROR;
  ++yynerrs;

  /* Do not reclaim the symbols of the rule whose action triggered
     this YYERROR.  */
  YYPOPSTACK (yylen);
  yylen = 0;
  YY_STACK_PRINT (yyss, yyssp);
  yystate = *yyssp;
  goto yyerrlab1;


/*-------------------------------------------------------------.
| yyerrlab1 -- common code for both syntax error and YYERROR.  |
`-------------------------------------------------------------*/
yyerrlab1:
  yyerrstatus = 3;      /* Each real token shifted decrements this.  */

  /* Pop stack until we find a state that shifts the error token.  */
  for (;;)
    {
      yyn = yypact[yystate];
      if (!yypact_value_is_default (yyn))
        {
          yyn += YYSYMBOL_YYerror;
          if (0 <= yyn && yyn <= YYLAST && yycheck[yyn] == YYSYMBOL_YYerror)
            {
              yyn = yytable[yyn];
              if (0 < yyn)
                break;
            }
        }

      /* Pop the current state because it cannot handle the error token.  */
      if (yyssp == yyss)
        YYABORT;


      yydestruct ("Error: popping",
                  YY_ACCESSING_SYMBOL (yystate), yyvsp);
      YYPOPSTACK (1);
      yystate = *yyssp;
      YY_STACK_PRINT (yyss, yyssp);
    }

  YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
  *++yyvsp = yylval;
  YY_IGNORE_MAYBE_UNINITIALIZED_END


  /* Shift the error token.  */
  YY_SYMBOL_PRINT ("Shifting", YY_ACCESSING_SYMBOL (yyn), yyvsp, yylsp);

  yystate = yyn;
  goto yynewstate;


/*-------------------------------------.
| yyacceptlab -- YYACCEPT comes here.  |
`-------------------------------------*/
yyacceptlab:
  yyresult = 0;
  goto yyreturnlab;


/*-----------------------------------.
| yyabortlab -- YYABORT comes here.  |
`-----------------------------------*/
yyabortlab:
  yyresult = 1;
  goto yyreturnlab;


/*-----------------------------------------------------------.
| yyexhaustedlab -- YYNOMEM (memory exhaustion) comes here.  |
`-----------------------------------------------------------*/
yyexhaustedlab:
  yyerror (YY_("memory exhausted"));
  yyresult = 2;
  goto yyreturnlab;


/*----------------------------------------------------------.
| yyreturnlab -- parsing is finished, clean up and return.  |
`----------------------------------------------------------*/
yyreturnlab:
  if (yychar != YYEMPTY)
    {
      /* Make sure we have latest lookahead translation.  See comments at
         user semantic actions for why this is necessary.  */
      yytoken = YYTRANSLATE (yychar);
      yydestruct ("Cleanup: discarding lookahead",
                  yytoken, &yylval);
    }
  /* Do not reclaim the symbols of the rule whose action triggered
     this YYABORT or YYACCEPT.  */
  YYPOPSTACK (yylen);
  YY_STACK_PRINT (yyss, yyssp);
  while (yyssp != yyss)
    {
      yydestruct ("Cleanup: popping",
                  YY_ACCESSING_SYMBOL (+*yyssp), yyvsp);
      YYPOPSTACK (1);
    }
#ifndef yyoverflow
  if (yyss != yyssa)
    YYSTACK_FREE (yyss);
#endif

  return yyresult;
}

#line 825 "/home/captainnothing/OpenROAD/src/sta/parasitics/SpefParse.yy"

