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
#define yyparse         VerilogParse_parse
#define yylex           VerilogParse_lex
#define yyerror         VerilogParse_error
#define yydebug         VerilogParse_debug
#define yynerrs         VerilogParse_nerrs
#define yylval          VerilogParse_lval
#define yychar          VerilogParse_char

/* First part of user prologue.  */
#line 1 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"


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

#include <cstdlib>
#include <string>
#include <iostream>

#include "PortDirection.hh"
#include "verilog/VerilogReaderPvt.hh"
#include "VerilogReader.hh"

int VerilogLex_lex();
#define VerilogParse_lex VerilogLex_lex
// Use yacc generated parser errors.
#define YYERROR_VERBOSE


#line 111 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"

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

#include "VerilogParse.hh"
/* Symbol kind.  */
enum yysymbol_kind_t
{
  YYSYMBOL_YYEMPTY = -2,
  YYSYMBOL_YYEOF = 0,                      /* "end of file"  */
  YYSYMBOL_YYerror = 1,                    /* error  */
  YYSYMBOL_YYUNDEF = 2,                    /* "invalid token"  */
  YYSYMBOL_INT = 3,                        /* INT  */
  YYSYMBOL_CONSTANT = 4,                   /* CONSTANT  */
  YYSYMBOL_ID = 5,                         /* ID  */
  YYSYMBOL_STRING = 6,                     /* STRING  */
  YYSYMBOL_MODULE = 7,                     /* MODULE  */
  YYSYMBOL_ENDMODULE = 8,                  /* ENDMODULE  */
  YYSYMBOL_ASSIGN = 9,                     /* ASSIGN  */
  YYSYMBOL_PARAMETER = 10,                 /* PARAMETER  */
  YYSYMBOL_DEFPARAM = 11,                  /* DEFPARAM  */
  YYSYMBOL_WIRE = 12,                      /* WIRE  */
  YYSYMBOL_WAND = 13,                      /* WAND  */
  YYSYMBOL_WOR = 14,                       /* WOR  */
  YYSYMBOL_TRI = 15,                       /* TRI  */
  YYSYMBOL_INPUT = 16,                     /* INPUT  */
  YYSYMBOL_OUTPUT = 17,                    /* OUTPUT  */
  YYSYMBOL_INOUT = 18,                     /* INOUT  */
  YYSYMBOL_SUPPLY1 = 19,                   /* SUPPLY1  */
  YYSYMBOL_SUPPLY0 = 20,                   /* SUPPLY0  */
  YYSYMBOL_REG = 21,                       /* REG  */
  YYSYMBOL_ATTRIBUTE_OPEN = 22,            /* ATTRIBUTE_OPEN  */
  YYSYMBOL_ATTRIBUTE_CLOSED = 23,          /* ATTRIBUTE_CLOSED  */
  YYSYMBOL_24_ = 24,                       /* '-'  */
  YYSYMBOL_25_ = 25,                       /* '+'  */
  YYSYMBOL_26_ = 26,                       /* '*'  */
  YYSYMBOL_27_ = 27,                       /* '/'  */
  YYSYMBOL_NEG = 28,                       /* NEG  */
  YYSYMBOL_29_ = 29,                       /* ';'  */
  YYSYMBOL_30_ = 30,                       /* '('  */
  YYSYMBOL_31_ = 31,                       /* ')'  */
  YYSYMBOL_32_ = 32,                       /* ','  */
  YYSYMBOL_33_ = 33,                       /* '.'  */
  YYSYMBOL_34_ = 34,                       /* '{'  */
  YYSYMBOL_35_ = 35,                       /* '}'  */
  YYSYMBOL_36_ = 36,                       /* '['  */
  YYSYMBOL_37_ = 37,                       /* ':'  */
  YYSYMBOL_38_ = 38,                       /* ']'  */
  YYSYMBOL_39_ = 39,                       /* '='  */
  YYSYMBOL_40_ = 40,                       /* '`'  */
  YYSYMBOL_41_ = 41,                       /* '#'  */
  YYSYMBOL_YYACCEPT = 42,                  /* $accept  */
  YYSYMBOL_file = 43,                      /* file  */
  YYSYMBOL_modules = 44,                   /* modules  */
  YYSYMBOL_module_begin = 45,              /* module_begin  */
  YYSYMBOL_46_1 = 46,                      /* @1  */
  YYSYMBOL_module = 47,                    /* module  */
  YYSYMBOL_port_list = 48,                 /* port_list  */
  YYSYMBOL_port = 49,                      /* port  */
  YYSYMBOL_port_expr = 50,                 /* port_expr  */
  YYSYMBOL_port_refs = 51,                 /* port_refs  */
  YYSYMBOL_port_ref = 52,                  /* port_ref  */
  YYSYMBOL_port_dcls = 53,                 /* port_dcls  */
  YYSYMBOL_port_dcl = 54,                  /* port_dcl  */
  YYSYMBOL_55_2 = 55,                      /* @2  */
  YYSYMBOL_56_3 = 56,                      /* @3  */
  YYSYMBOL_port_dcl_type = 57,             /* port_dcl_type  */
  YYSYMBOL_stmts = 58,                     /* stmts  */
  YYSYMBOL_stmt = 59,                      /* stmt  */
  YYSYMBOL_stmt_seq = 60,                  /* stmt_seq  */
  YYSYMBOL_parameter = 61,                 /* parameter  */
  YYSYMBOL_parameter_dcls = 62,            /* parameter_dcls  */
  YYSYMBOL_parameter_dcl = 63,             /* parameter_dcl  */
  YYSYMBOL_parameter_expr = 64,            /* parameter_expr  */
  YYSYMBOL_defparam = 65,                  /* defparam  */
  YYSYMBOL_param_values = 66,              /* param_values  */
  YYSYMBOL_param_value = 67,               /* param_value  */
  YYSYMBOL_declaration = 68,               /* declaration  */
  YYSYMBOL_69_4 = 69,                      /* @4  */
  YYSYMBOL_70_5 = 70,                      /* @5  */
  YYSYMBOL_dcl_type = 71,                  /* dcl_type  */
  YYSYMBOL_dcl_args = 72,                  /* dcl_args  */
  YYSYMBOL_dcl_arg = 73,                   /* dcl_arg  */
  YYSYMBOL_continuous_assign = 74,         /* continuous_assign  */
  YYSYMBOL_net_assignments = 75,           /* net_assignments  */
  YYSYMBOL_net_assignment = 76,            /* net_assignment  */
  YYSYMBOL_77_6 = 77,                      /* @6  */
  YYSYMBOL_net_assign_lhs = 78,            /* net_assign_lhs  */
  YYSYMBOL_instance = 79,                  /* instance  */
  YYSYMBOL_80_7 = 80,                      /* @7  */
  YYSYMBOL_81_8 = 81,                      /* @8  */
  YYSYMBOL_parameter_values = 82,          /* parameter_values  */
  YYSYMBOL_parameter_exprs = 83,           /* parameter_exprs  */
  YYSYMBOL_inst_pins = 84,                 /* inst_pins  */
  YYSYMBOL_inst_ordered_pins = 85,         /* inst_ordered_pins  */
  YYSYMBOL_inst_named_pins = 86,           /* inst_named_pins  */
  YYSYMBOL_inst_named_pin = 87,            /* inst_named_pin  */
  YYSYMBOL_named_pin_net_expr = 88,        /* named_pin_net_expr  */
  YYSYMBOL_net_named = 89,                 /* net_named  */
  YYSYMBOL_net_scalar = 90,                /* net_scalar  */
  YYSYMBOL_net_bit_select = 91,            /* net_bit_select  */
  YYSYMBOL_net_part_select = 92,           /* net_part_select  */
  YYSYMBOL_net_constant = 93,              /* net_constant  */
  YYSYMBOL_net_expr_concat = 94,           /* net_expr_concat  */
  YYSYMBOL_net_exprs = 95,                 /* net_exprs  */
  YYSYMBOL_net_expr = 96,                  /* net_expr  */
  YYSYMBOL_attribute_instance_seq = 97,    /* attribute_instance_seq  */
  YYSYMBOL_attribute_instance = 98,        /* attribute_instance  */
  YYSYMBOL_attr_specs = 99,                /* attr_specs  */
  YYSYMBOL_attr_spec = 100,                /* attr_spec  */
  YYSYMBOL_attr_spec_value = 101           /* attr_spec_value  */
};
typedef enum yysymbol_kind_t yysymbol_kind_t;


/* Second part of user prologue.  */
#line 89 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"


#line 249 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"


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
#define YYFINAL  3
/* YYLAST -- Last index in YYTABLE.  */
#define YYLAST   355

/* YYNTOKENS -- Number of terminals.  */
#define YYNTOKENS  42
/* YYNNTS -- Number of nonterminals.  */
#define YYNNTS  60
/* YYNRULES -- Number of rules.  */
#define YYNRULES  142
/* YYNSTATES -- Number of states.  */
#define YYNSTATES  260

/* YYMAXUTOK -- Last valid token kind.  */
#define YYMAXUTOK   279


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
       2,     2,     2,     2,     2,    41,     2,     2,     2,     2,
      30,    31,    26,    25,    32,    24,    33,    27,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,    37,    29,
       2,    39,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,    36,     2,    38,     2,     2,    40,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,    34,     2,    35,     2,     2,     2,     2,
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
      15,    16,    17,    18,    19,    20,    21,    22,    23,    28
};

#if YYDEBUG
/* YYRLINE[YYN] -- Source line where rule number YYN was defined.  */
static const yytype_int16 yyrline[] =
{
       0,    95,    95,    98,   100,   104,   104,   109,   111,   113,
     115,   120,   124,   129,   130,   132,   137,   138,   142,   146,
     151,   152,   153,   157,   161,   165,   175,   174,   178,   177,
     183,   184,   185,   186,   187,   188,   189,   190,   195,   196,
     198,   208,   209,   210,   211,   212,   217,   222,   224,   229,
     231,   236,   240,   248,   252,   256,   260,   261,   263,   265,
     267,   269,   271,   276,   281,   283,   288,   292,   300,   300,
     302,   302,   308,   309,   310,   311,   312,   313,   314,   315,
     316,   320,   324,   331,   333,   338,   343,   347,   352,   352,
     357,   358,   362,   362,   364,   364,   370,   374,   375,   377,
     382,   383,   384,   389,   393,   399,   403,   411,   413,   415,
     417,   420,   422,   425,   427,   432,   433,   434,   438,   439,
     440,   444,   449,   454,   459,   464,   469,   473,   478,   479,
     480,   481,   482,   487,   488,   493,   498,   502,   507,   511,
     519,   521,   523
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
  "\"end of file\"", "error", "\"invalid token\"", "INT", "CONSTANT",
  "ID", "STRING", "MODULE", "ENDMODULE", "ASSIGN", "PARAMETER", "DEFPARAM",
  "WIRE", "WAND", "WOR", "TRI", "INPUT", "OUTPUT", "INOUT", "SUPPLY1",
  "SUPPLY0", "REG", "ATTRIBUTE_OPEN", "ATTRIBUTE_CLOSED", "'-'", "'+'",
  "'*'", "'/'", "NEG", "';'", "'('", "')'", "','", "'.'", "'{'", "'}'",
  "'['", "':'", "']'", "'='", "'`'", "'#'", "$accept", "file", "modules",
  "module_begin", "@1", "module", "port_list", "port", "port_expr",
  "port_refs", "port_ref", "port_dcls", "port_dcl", "@2", "@3",
  "port_dcl_type", "stmts", "stmt", "stmt_seq", "parameter",
  "parameter_dcls", "parameter_dcl", "parameter_expr", "defparam",
  "param_values", "param_value", "declaration", "@4", "@5", "dcl_type",
  "dcl_args", "dcl_arg", "continuous_assign", "net_assignments",
  "net_assignment", "@6", "net_assign_lhs", "instance", "@7", "@8",
  "parameter_values", "parameter_exprs", "inst_pins", "inst_ordered_pins",
  "inst_named_pins", "inst_named_pin", "named_pin_net_expr", "net_named",
  "net_scalar", "net_bit_select", "net_part_select", "net_constant",
  "net_expr_concat", "net_exprs", "net_expr", "attribute_instance_seq",
  "attribute_instance", "attr_specs", "attr_spec", "attr_spec_value", YY_NULLPTR
};

static const char *
yysymbol_name (yysymbol_kind_t yysymbol)
{
  return yytname[yysymbol];
}
#endif

#define YYPACT_NINF (-122)

#define yypact_value_is_default(Yyn) \
  ((Yyn) == YYPACT_NINF)

#define YYTABLE_NINF (-134)

#define yytable_value_is_error(Yyn) \
  0

/* YYPACT[STATE-NUM] -- Index in YYTABLE of the portion describing
   STATE-NUM.  */
static const yytype_int16 yypact[] =
{
    -122,    17,    39,  -122,  -122,    16,  -122,    26,    36,  -122,
    -122,     4,   -10,  -122,    74,   118,  -122,    26,  -122,   110,
    -122,  -122,  -122,  -122,  -122,   201,    30,    41,    79,   103,
     246,  -122,  -122,  -122,   249,  -122,  -122,  -122,  -122,   134,
      85,  -122,    11,    10,   129,  -122,  -122,  -122,  -122,  -122,
    -122,  -122,   164,   133,  -122,   102,   127,  -122,   109,    57,
     113,    15,   142,     9,   104,   125,  -122,    71,   156,  -122,
    -122,  -122,  -122,  -122,  -122,  -122,   124,   172,   163,  -122,
     128,   168,  -122,   149,  -122,  -122,  -122,  -122,  -122,  -122,
    -122,  -122,  -122,   205,   260,   234,    78,   103,  -122,  -122,
    -122,  -122,   169,  -122,  -122,  -122,  -122,  -122,  -122,  -122,
    -122,    15,   222,  -122,  -122,  -122,  -122,  -122,  -122,   190,
    -122,  -122,    11,   165,    23,   200,  -122,   256,    31,  -122,
     129,   290,   192,    15,   271,   305,  -122,  -122,  -122,   278,
    -122,   254,   274,  -122,   307,    71,  -122,  -122,    71,  -122,
    -122,  -122,  -122,   115,   115,   306,   146,   309,  -122,  -122,
     146,  -122,   283,   284,   310,   195,  -122,   313,   279,  -122,
    -122,  -122,   281,  -122,  -122,  -122,   122,  -122,   115,   115,
     115,   115,   282,    60,    83,   289,  -122,    15,   285,  -122,
     318,  -122,   273,   273,  -122,  -122,   256,   319,   292,   293,
     294,  -122,  -122,    83,   146,   270,    60,  -122,   324,   291,
     197,   -12,   299,    71,   297,   196,  -122,   115,   300,   295,
      15,  -122,    64,   329,  -122,  -122,  -122,  -122,   146,   308,
      15,  -122,    70,  -122,   303,  -122,  -122,  -122,   266,  -122,
     228,  -122,   332,  -122,   333,   311,  -122,   268,   301,    68,
     312,   314,  -122,   315,  -122,   106,  -122,  -122,   316,  -122
};

/* YYDEFACT[STATE-NUM] -- Default reduction number in state STATE-NUM.
   Performed when YYTABLE does not specify something else to do.  Zero
   means the default is an error.  */
static const yytype_uint8 yydefact[] =
{
       3,     0,   133,     1,     4,     0,     5,     0,     0,   134,
       6,   138,     0,   136,     0,     0,   135,     0,    38,   133,
     142,   140,   141,   139,   137,     0,   121,     0,     0,     0,
       0,    11,    13,    16,     0,    23,    20,    21,    22,     0,
       0,     7,     0,     0,     0,    39,    40,    41,    42,    43,
      46,    44,     0,     0,    38,     0,     0,    18,     0,     0,
       0,   133,    30,    35,    32,    26,    45,     0,     0,    86,
      88,    90,   118,   119,   120,    91,     0,     0,     0,    49,
       0,     0,    64,    92,    79,    78,    80,    77,    72,    74,
      73,    76,    75,    68,     0,     0,     0,     0,    17,    38,
      12,    38,    83,    24,    25,    84,    31,    36,    37,    34,
      33,     0,     0,   124,   128,   129,   130,   131,   132,     0,
     126,    85,     0,     0,     0,     0,    47,     0,     0,    63,
       0,     0,     0,     0,     0,     0,   122,     8,    14,     0,
      19,     0,     0,    27,     0,     0,   125,    87,     0,    56,
      55,    53,    52,     0,     0,     0,    51,     0,    50,    67,
      66,    65,     0,     0,     0,     0,    81,     0,     0,    15,
       9,    10,     0,   127,    89,    57,     0,    54,     0,     0,
       0,     0,     0,   100,     0,     0,    69,     0,     0,   123,
       0,    62,    59,    58,    60,    61,     0,     0,     0,   101,
     102,   105,   103,     0,    97,     0,   100,    82,     0,     0,
       0,     0,     0,     0,     0,     0,    96,     0,     0,     0,
       0,    48,     0,     0,    93,   104,   106,    98,    99,     0,
       0,    29,     0,   107,     0,   115,   116,   117,     0,    95,
       0,   108,     0,   110,     0,     0,    71,     0,     0,     0,
       0,     0,   111,     0,   109,     0,   112,   113,     0,   114
};

/* YYPGOTO[NTERM-NUM].  */
static const yytype_int16 yypgoto[] =
{
    -122,  -122,  -122,  -122,  -122,  -122,  -122,   286,   242,  -122,
     -23,  -122,   287,  -122,  -122,  -122,   -45,  -122,  -122,  -122,
     144,   215,  -121,  -122,  -122,   219,  -122,  -122,  -122,  -122,
     120,   -60,  -122,  -122,   -37,  -122,  -122,  -122,  -122,  -122,
    -122,   148,   147,  -122,  -122,   138,  -122,  -122,   -17,   -15,
     -19,   132,   -48,  -122,   -56,     6,  -122,  -122,   338,  -122
};

/* YYDEFGOTO[NTERM-NUM].  */
static const yytype_uint8 yydefgoto[] =
{
       0,     1,     2,     8,    10,     4,    30,    31,    32,    56,
      33,    34,    35,   111,   112,    65,    25,    45,    46,    47,
      78,    79,   204,    48,    81,    82,    49,   133,   134,    93,
     165,   166,    50,    68,   105,   123,    70,    51,   131,   132,
     164,   205,   198,   199,   200,   201,   234,    71,    72,    73,
      74,   117,    75,   119,   202,    52,     9,    12,    13,    23
};

/* YYTABLE[YYPACT[STATE-NUM]] -- What to do in state STATE-NUM.  If
   positive, shift that token.  If negative, reduce the rule whose
   number is the opposite.  If YYTABLE_NINF, syntax error.  */
static const yytype_int16 yytable[] =
{
      38,   104,    36,   156,    37,    69,    57,   160,     5,    95,
      38,   120,    36,    16,    37,    76,    26,     3,   222,   118,
     102,   107,    17,     6,   223,    39,   149,   150,   151,   152,
     108,    11,   175,   176,   149,   150,   151,   159,     7,    -2,
      38,    14,    36,    15,    37,    67,    77,   153,   116,    67,
     114,   143,   115,   154,   141,   153,   142,   192,   193,   194,
     195,   154,    26,   155,   113,    26,    53,    39,   113,   232,
      54,   155,   113,    26,   140,   113,    26,    38,    38,    36,
      36,    37,    37,    26,    55,   147,   149,   150,   151,   173,
      28,    29,   174,   197,    67,   233,   228,   118,    67,   252,
     118,   241,    67,    18,    19,    67,   242,   153,    26,   138,
     113,    26,    29,   154,    66,    26,   109,   203,   149,   150,
     151,    20,    21,   155,    22,   110,   116,   207,   114,   116,
     115,   114,    96,   115,    80,   118,    94,   257,    99,   153,
      67,    27,   101,    28,    29,   154,   178,   179,   180,   181,
      62,    63,    64,   191,   106,   155,     7,   225,   118,    97,
     231,   -28,    98,   124,   116,   118,   114,   128,   115,    83,
     178,   179,   180,   181,   237,   125,    84,    85,    86,    87,
      88,    89,    90,    91,    92,   121,     7,   116,   122,   114,
     -94,   115,   126,   253,   116,   127,   114,   129,   115,   258,
     130,   118,    40,   235,   148,    53,  -133,   118,  -121,    41,
      42,    43,    44,  -133,  -133,  -133,  -133,  -133,  -133,  -133,
    -133,  -133,   145,  -133,   186,   146,   221,   187,   217,   127,
     116,   227,   114,   163,   115,    40,   116,   157,   114,  -133,
     115,   -70,   137,    42,    43,    44,  -133,  -133,  -133,  -133,
    -133,  -133,  -133,  -133,  -133,    40,  -133,   246,   144,  -133,
     187,    76,   170,    42,    43,    44,  -133,  -133,  -133,  -133,
    -133,  -133,  -133,  -133,  -133,    40,  -133,    58,    59,  -133,
      60,    61,   171,    42,    43,    44,  -133,  -133,  -133,  -133,
    -133,  -133,  -133,  -133,  -133,   162,  -133,   135,   136,   180,
     181,   216,   217,   244,   245,   135,   250,   167,   168,   169,
     172,   177,   182,   183,   184,   185,   188,   189,   190,   206,
     196,   209,   208,   212,   211,   213,   214,   219,   224,   220,
     197,   229,   238,   230,   243,   247,   248,   239,   139,   251,
     210,   249,   158,   254,   255,   100,   256,   259,   103,   161,
     240,   215,   226,   218,   236,    24
};

static const yytype_uint8 yycheck[] =
{
      19,    61,    19,   124,    19,    42,    29,   128,     2,    54,
      29,    67,    29,    23,    29,     5,     5,     0,    30,    67,
       5,    12,    32,     7,    36,    19,     3,     4,     5,     6,
      21,     5,   153,   154,     3,     4,     5,     6,    22,     0,
      59,     5,    59,    39,    59,    34,    36,    24,    67,    34,
      67,   111,    67,    30,    99,    24,   101,   178,   179,   180,
     181,    30,     5,    40,     4,     5,    36,    61,     4,     5,
      29,    40,     4,     5,    97,     4,     5,    96,    97,    96,
      97,    96,    97,     5,     5,   122,     3,     4,     5,   145,
      33,    34,   148,    33,    34,    31,   217,   145,    34,    31,
     148,    31,    34,    29,    30,    34,    36,    24,     5,    31,
       4,     5,    34,    30,    29,     5,    12,    34,     3,     4,
       5,     3,     4,    40,     6,    21,   145,   187,   145,   148,
     145,   148,    30,   148,     5,   183,     3,    31,    29,    24,
      34,    31,    29,    33,    34,    30,    24,    25,    26,    27,
      16,    17,    18,    31,    12,    40,    22,   213,   206,    32,
     220,    36,    35,    39,   183,   213,   183,    39,   183,     5,
      24,    25,    26,    27,   222,     3,    12,    13,    14,    15,
      16,    17,    18,    19,    20,    29,    22,   206,    32,   206,
      41,   206,    29,   249,   213,    32,   213,    29,   213,   255,
      32,   249,     1,   222,    39,    36,     5,   255,    39,     8,
       9,    10,    11,    12,    13,    14,    15,    16,    17,    18,
      19,    20,    32,    22,    29,    35,    29,    32,    32,    32,
     249,    35,   249,    41,   249,     1,   255,    37,   255,     5,
     255,    36,     8,     9,    10,    11,    12,    13,    14,    15,
      16,    17,    18,    19,    20,     1,    22,    29,    36,     5,
      32,     5,     8,     9,    10,    11,    12,    13,    14,    15,
      16,    17,    18,    19,    20,     1,    22,    31,    32,     5,
      31,    32,     8,     9,    10,    11,    12,    13,    14,    15,
      16,    17,    18,    19,    20,     5,    22,    37,    38,    26,
      27,    31,    32,    37,    38,    37,    38,    36,     3,    31,
       3,     5,     3,    30,    30,     5,     3,    38,    37,    30,
      38,     3,    37,    31,     5,    32,    32,     3,    29,    38,
      33,    31,     3,    38,    31,     3,     3,    29,    96,    38,
     196,    30,   127,    31,    30,    59,    31,    31,    61,   130,
     230,   203,   214,   206,   222,    17
};

/* YYSTOS[STATE-NUM] -- The symbol kind of the accessing symbol of
   state STATE-NUM.  */
static const yytype_int8 yystos[] =
{
       0,    43,    44,     0,    47,    97,     7,    22,    45,    98,
      46,     5,    99,   100,     5,    39,    23,    32,    29,    30,
       3,     4,     6,   101,   100,    58,     5,    31,    33,    34,
      48,    49,    50,    52,    53,    54,    90,    91,    92,    97,
       1,     8,     9,    10,    11,    59,    60,    61,    65,    68,
      74,    79,    97,    36,    29,     5,    51,    52,    31,    32,
      31,    32,    16,    17,    18,    57,    29,    34,    75,    76,
      78,    89,    90,    91,    92,    94,     5,    36,    62,    63,
       5,    66,    67,     5,    12,    13,    14,    15,    16,    17,
      18,    19,    20,    71,     3,    58,    30,    32,    35,    29,
      49,    29,     5,    54,    73,    76,    12,    12,    21,    12,
      21,    55,    56,     4,    90,    91,    92,    93,    94,    95,
      96,    29,    32,    77,    39,     3,    29,    32,    39,    29,
      32,    80,    81,    69,    70,    37,    38,     8,    31,    50,
      52,    58,    58,    73,    36,    32,    35,    76,    39,     3,
       4,     5,     6,    24,    30,    40,    64,    37,    63,     6,
      64,    67,     5,    41,    82,    72,    73,    36,     3,    31,
       8,     8,     3,    96,    96,    64,    64,     5,    24,    25,
      26,    27,     3,    30,    30,     5,    29,    32,     3,    38,
      37,    31,    64,    64,    64,    64,    38,    33,    84,    85,
      86,    87,    96,    34,    64,    83,    30,    73,    37,     3,
      62,     5,    31,    32,    32,    83,    31,    32,    84,     3,
      38,    29,    30,    36,    29,    96,    87,    35,    64,    31,
      38,    73,     5,    31,    88,    92,    93,    94,     3,    29,
      72,    31,    36,    31,    37,    38,    29,     3,     3,    30,
      38,    38,    31,    96,    31,    30,    31,    31,    96,    31
};

/* YYR1[RULE-NUM] -- Symbol kind of the left-hand side of rule RULE-NUM.  */
static const yytype_int8 yyr1[] =
{
       0,    42,    43,    44,    44,    46,    45,    47,    47,    47,
      47,    48,    48,    49,    49,    49,    50,    50,    51,    51,
      52,    52,    52,    53,    53,    53,    55,    54,    56,    54,
      57,    57,    57,    57,    57,    57,    57,    57,    58,    58,
      58,    59,    59,    59,    59,    59,    60,    61,    61,    62,
      62,    63,    63,    64,    64,    64,    64,    64,    64,    64,
      64,    64,    64,    65,    66,    66,    67,    67,    69,    68,
      70,    68,    71,    71,    71,    71,    71,    71,    71,    71,
      71,    72,    72,    73,    73,    74,    75,    75,    77,    76,
      78,    78,    80,    79,    81,    79,    82,    83,    83,    83,
      84,    84,    84,    85,    85,    86,    86,    87,    87,    87,
      87,    87,    87,    87,    87,    88,    88,    88,    89,    89,
      89,    90,    91,    92,    93,    94,    95,    95,    96,    96,
      96,    96,    96,    97,    97,    98,    99,    99,   100,   100,
     101,   101,   101
};

/* YYR2[RULE-NUM] -- Number of symbols on the right-hand side of rule RULE-NUM.  */
static const yytype_int8 yyr2[] =
{
       0,     2,     1,     0,     2,     0,     2,     6,     8,     9,
       9,     1,     3,     1,     4,     5,     1,     3,     1,     3,
       1,     1,     1,     1,     3,     3,     0,     4,     0,     9,
       1,     2,     1,     2,     2,     1,     2,     2,     0,     2,
       2,     1,     1,     1,     1,     2,     1,     3,     8,     1,
       3,     3,     3,     1,     2,     1,     1,     2,     3,     3,
       3,     3,     3,     3,     1,     3,     3,     3,     0,     5,
       0,    10,     1,     1,     1,     1,     1,     1,     1,     1,
       1,     1,     3,     1,     1,     3,     1,     3,     0,     4,
       1,     1,     0,     8,     0,     9,     4,     1,     3,     3,
       0,     1,     1,     1,     3,     1,     3,     4,     5,     8,
       5,     7,     8,     9,    10,     1,     1,     1,     1,     1,
       1,     1,     4,     6,     1,     3,     1,     3,     1,     1,
       1,     1,     1,     0,     2,     3,     1,     3,     1,     3,
       1,     1,     1
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
  case 5: /* @1: %empty  */
#line 104 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
               { (yyval.ival) = sta::verilog_reader->line(); }
#line 1405 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 6: /* module_begin: MODULE @1  */
#line 105 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.ival) = (yyvsp[0].ival); }
#line 1411 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 7: /* module: attribute_instance_seq module_begin ID ';' stmts ENDMODULE  */
#line 110 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { sta::verilog_reader->makeModule((yyvsp[-3].string), new sta::VerilogNetSeq, (yyvsp[-1].stmt_seq), (yyvsp[-5].attribute_stmt_seq), (yyvsp[-4].ival));}
#line 1417 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 8: /* module: attribute_instance_seq module_begin ID '(' ')' ';' stmts ENDMODULE  */
#line 112 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { sta::verilog_reader->makeModule((yyvsp[-5].string), new sta::VerilogNetSeq, (yyvsp[-1].stmt_seq), (yyvsp[-7].attribute_stmt_seq), (yyvsp[-6].ival));}
#line 1423 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 9: /* module: attribute_instance_seq module_begin ID '(' port_list ')' ';' stmts ENDMODULE  */
#line 114 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { sta::verilog_reader->makeModule((yyvsp[-6].string), (yyvsp[-4].nets), (yyvsp[-1].stmt_seq), (yyvsp[-8].attribute_stmt_seq), (yyvsp[-7].ival)); }
#line 1429 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 10: /* module: attribute_instance_seq module_begin ID '(' port_dcls ')' ';' stmts ENDMODULE  */
#line 116 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { sta::verilog_reader->makeModule((yyvsp[-6].string), (yyvsp[-4].stmt_seq), (yyvsp[-1].stmt_seq), (yyvsp[-8].attribute_stmt_seq), (yyvsp[-7].ival)); }
#line 1435 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 11: /* port_list: port  */
#line 121 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.nets) = new sta::VerilogNetSeq;
	  (yyval.nets)->push_back((yyvsp[0].net));
	}
#line 1443 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 12: /* port_list: port_list ',' port  */
#line 125 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyvsp[-2].nets)->push_back((yyvsp[0].net)); }
#line 1449 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 14: /* port: '.' ID '(' ')'  */
#line 131 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.net)=sta::verilog_reader->makeNetNamedPortRefScalar((yyvsp[-2].string), NULL);}
#line 1455 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 15: /* port: '.' ID '(' port_expr ')'  */
#line 133 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.net)=sta::verilog_reader->makeNetNamedPortRefScalar((yyvsp[-3].string), (yyvsp[-1].net));}
#line 1461 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 17: /* port_expr: '{' port_refs '}'  */
#line 139 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.net) = sta::verilog_reader->makeNetConcat((yyvsp[-1].nets)); }
#line 1467 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 18: /* port_refs: port_ref  */
#line 143 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.nets) = new sta::VerilogNetSeq;
	  (yyval.nets)->push_back((yyvsp[0].net));
	}
#line 1475 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 19: /* port_refs: port_refs ',' port_ref  */
#line 147 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyvsp[-2].nets)->push_back((yyvsp[0].net)); }
#line 1481 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 23: /* port_dcls: port_dcl  */
#line 158 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.stmt_seq) = new sta::VerilogStmtSeq;
	  (yyval.stmt_seq)->push_back((yyvsp[0].stmt));
	}
#line 1489 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 24: /* port_dcls: port_dcls ',' port_dcl  */
#line 162 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.stmt_seq) = (yyvsp[-2].stmt_seq);
	  (yyvsp[-2].stmt_seq)->push_back((yyvsp[0].stmt));
	}
#line 1497 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 25: /* port_dcls: port_dcls ',' dcl_arg  */
#line 166 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        {
	  sta::VerilogDcl *dcl = dynamic_cast<sta::VerilogDcl*>((yyvsp[-2].stmt_seq)->back());
	  dcl->appendArg((yyvsp[0].dcl_arg));
	  (yyval.stmt_seq) = (yyvsp[-2].stmt_seq);
	}
#line 1507 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 26: /* @2: %empty  */
#line 175 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.ival) = sta::verilog_reader->line(); }
#line 1513 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 27: /* port_dcl: attribute_instance_seq port_dcl_type @2 dcl_arg  */
#line 176 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.stmt) = sta::verilog_reader->makeDcl((yyvsp[-2].port_type), (yyvsp[0].dcl_arg), (yyvsp[-3].attribute_stmt_seq), (yyvsp[-1].ival)); }
#line 1519 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 28: /* @3: %empty  */
#line 178 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
  { (yyval.ival) = sta::verilog_reader->line(); }
#line 1525 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 29: /* port_dcl: attribute_instance_seq port_dcl_type @3 '[' INT ':' INT ']' dcl_arg  */
#line 179 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.stmt) = sta::verilog_reader->makeDclBus((yyvsp[-7].port_type), (yyvsp[-4].ival), (yyvsp[-2].ival), (yyvsp[0].dcl_arg), (yyvsp[-8].attribute_stmt_seq), (yyvsp[-6].ival)); }
#line 1531 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 30: /* port_dcl_type: INPUT  */
#line 183 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
              { (yyval.port_type) = sta::PortDirection::input(); }
#line 1537 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 31: /* port_dcl_type: INPUT WIRE  */
#line 184 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
                   { (yyval.port_type) = sta::PortDirection::input(); }
#line 1543 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 32: /* port_dcl_type: INOUT  */
#line 185 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
              { (yyval.port_type) = sta::PortDirection::bidirect(); }
#line 1549 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 33: /* port_dcl_type: INOUT REG  */
#line 186 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
                  { (yyval.port_type) = sta::PortDirection::bidirect(); }
#line 1555 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 34: /* port_dcl_type: INOUT WIRE  */
#line 187 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
                   { (yyval.port_type) = sta::PortDirection::bidirect(); }
#line 1561 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 35: /* port_dcl_type: OUTPUT  */
#line 188 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
               { (yyval.port_type) = sta::PortDirection::output(); }
#line 1567 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 36: /* port_dcl_type: OUTPUT WIRE  */
#line 189 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
                    { (yyval.port_type) = sta::PortDirection::output(); }
#line 1573 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 37: /* port_dcl_type: OUTPUT REG  */
#line 190 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
                   { (yyval.port_type) = sta::PortDirection::output(); }
#line 1579 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 38: /* stmts: %empty  */
#line 195 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.stmt_seq) = new sta::VerilogStmtSeq; }
#line 1585 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 39: /* stmts: stmts stmt  */
#line 197 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { if ((yyvsp[0].stmt)) (yyvsp[-1].stmt_seq)->push_back((yyvsp[0].stmt)); }
#line 1591 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 40: /* stmts: stmts stmt_seq  */
#line 200 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { sta::VerilogStmtSeq::Iterator iter((yyvsp[0].stmt_seq));
	  while (iter.hasNext())
	    (yyvsp[-1].stmt_seq)->push_back(iter.next());
	  delete (yyvsp[0].stmt_seq);
	}
#line 1601 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 45: /* stmt: error ';'  */
#line 213 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { yyerrok; (yyval.stmt) = NULL; }
#line 1607 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 47: /* parameter: PARAMETER parameter_dcls ';'  */
#line 223 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.stmt) = NULL; }
#line 1613 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 48: /* parameter: PARAMETER '[' INT ':' INT ']' parameter_dcls ';'  */
#line 225 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.stmt) = NULL; }
#line 1619 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 49: /* parameter_dcls: parameter_dcl  */
#line 230 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.stmt) = NULL; }
#line 1625 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 50: /* parameter_dcls: parameter_dcls ',' parameter_dcl  */
#line 232 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.stmt) = NULL; }
#line 1631 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 51: /* parameter_dcl: ID '=' parameter_expr  */
#line 237 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { sta::stringDelete((yyvsp[-2].string));
	  (yyval.stmt) = NULL;
	}
#line 1639 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 52: /* parameter_dcl: ID '=' STRING  */
#line 241 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { sta::stringDelete((yyvsp[-2].string));
	  sta::stringDelete((yyvsp[0].string));
	  (yyval.stmt) = NULL;
	}
#line 1648 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 53: /* parameter_expr: ID  */
#line 249 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { sta::stringDelete((yyvsp[0].string));
	  (yyval.ival) = 0;
	}
#line 1656 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 54: /* parameter_expr: '`' ID  */
#line 253 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { sta::stringDelete((yyvsp[0].string));
	  (yyval.ival) = 0;
	}
#line 1664 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 55: /* parameter_expr: CONSTANT  */
#line 257 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { sta::stringDelete((yyvsp[0].constant));
	  (yyval.ival) = 0;
	}
#line 1672 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 57: /* parameter_expr: '-' parameter_expr  */
#line 262 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.ival) = - (yyvsp[0].ival); }
#line 1678 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 58: /* parameter_expr: parameter_expr '+' parameter_expr  */
#line 264 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.ival) = (yyvsp[-2].ival) + (yyvsp[0].ival); }
#line 1684 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 59: /* parameter_expr: parameter_expr '-' parameter_expr  */
#line 266 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.ival) = (yyvsp[-2].ival) - (yyvsp[0].ival); }
#line 1690 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 60: /* parameter_expr: parameter_expr '*' parameter_expr  */
#line 268 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.ival) = (yyvsp[-2].ival) * (yyvsp[0].ival); }
#line 1696 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 61: /* parameter_expr: parameter_expr '/' parameter_expr  */
#line 270 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.ival) = (yyvsp[-2].ival) / (yyvsp[0].ival); }
#line 1702 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 62: /* parameter_expr: '(' parameter_expr ')'  */
#line 272 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.ival) = (yyvsp[-1].ival); }
#line 1708 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 63: /* defparam: DEFPARAM param_values ';'  */
#line 277 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.stmt) = NULL; }
#line 1714 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 64: /* param_values: param_value  */
#line 282 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.stmt) = NULL; }
#line 1720 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 65: /* param_values: param_values ',' param_value  */
#line 284 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.stmt) = NULL; }
#line 1726 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 66: /* param_value: ID '=' parameter_expr  */
#line 289 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { sta::stringDelete((yyvsp[-2].string));
	  (yyval.stmt) = NULL;
	}
#line 1734 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 67: /* param_value: ID '=' STRING  */
#line 293 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { sta::stringDelete((yyvsp[-2].string));
	  sta::stringDelete((yyvsp[0].string));
	  (yyval.stmt) = NULL;
	}
#line 1743 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 68: /* @4: %empty  */
#line 300 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
                                        { (yyval.ival) = sta::verilog_reader->line(); }
#line 1749 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 69: /* declaration: attribute_instance_seq dcl_type @4 dcl_args ';'  */
#line 301 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.stmt) = sta::verilog_reader->makeDcl((yyvsp[-3].port_type), (yyvsp[-1].dcl_arg_seq), (yyvsp[-4].attribute_stmt_seq), (yyvsp[-2].ival)); }
#line 1755 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 70: /* @5: %empty  */
#line 302 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
                                        { (yyval.ival) = sta::verilog_reader->line(); }
#line 1761 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 71: /* declaration: attribute_instance_seq dcl_type @5 '[' INT ':' INT ']' dcl_args ';'  */
#line 304 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.stmt) = sta::verilog_reader->makeDclBus((yyvsp[-8].port_type), (yyvsp[-5].ival), (yyvsp[-3].ival), (yyvsp[-1].dcl_arg_seq), (yyvsp[-9].attribute_stmt_seq),(yyvsp[-7].ival)); }
#line 1767 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 72: /* dcl_type: INPUT  */
#line 308 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
              { (yyval.port_type) = sta::PortDirection::input(); }
#line 1773 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 73: /* dcl_type: INOUT  */
#line 309 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
              { (yyval.port_type) = sta::PortDirection::bidirect(); }
#line 1779 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 74: /* dcl_type: OUTPUT  */
#line 310 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
               { (yyval.port_type) = sta::PortDirection::output(); }
#line 1785 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 75: /* dcl_type: SUPPLY0  */
#line 311 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
                { (yyval.port_type) = sta::PortDirection::ground(); }
#line 1791 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 76: /* dcl_type: SUPPLY1  */
#line 312 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
                { (yyval.port_type) = sta::PortDirection::power(); }
#line 1797 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 77: /* dcl_type: TRI  */
#line 313 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
            { (yyval.port_type) = sta::PortDirection::tristate(); }
#line 1803 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 78: /* dcl_type: WAND  */
#line 314 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
             { (yyval.port_type) = sta::PortDirection::internal(); }
#line 1809 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 79: /* dcl_type: WIRE  */
#line 315 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
             { (yyval.port_type) = sta::PortDirection::internal(); }
#line 1815 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 80: /* dcl_type: WOR  */
#line 316 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
            { (yyval.port_type) = sta::PortDirection::internal(); }
#line 1821 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 81: /* dcl_args: dcl_arg  */
#line 321 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.dcl_arg_seq) = new sta::VerilogDclArgSeq;
	  (yyval.dcl_arg_seq)->push_back((yyvsp[0].dcl_arg));
	}
#line 1829 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 82: /* dcl_args: dcl_args ',' dcl_arg  */
#line 325 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyvsp[-2].dcl_arg_seq)->push_back((yyvsp[0].dcl_arg));
	  (yyval.dcl_arg_seq) = (yyvsp[-2].dcl_arg_seq);
	}
#line 1837 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 83: /* dcl_arg: ID  */
#line 332 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.dcl_arg) = sta::verilog_reader->makeDclArg((yyvsp[0].string)); }
#line 1843 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 84: /* dcl_arg: net_assignment  */
#line 334 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.dcl_arg) = sta::verilog_reader->makeDclArg((yyvsp[0].assign)); }
#line 1849 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 85: /* continuous_assign: ASSIGN net_assignments ';'  */
#line 339 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.stmt_seq) = (yyvsp[-1].stmt_seq); }
#line 1855 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 86: /* net_assignments: net_assignment  */
#line 344 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.stmt_seq) = new sta::VerilogStmtSeq();
	  (yyval.stmt_seq)->push_back((yyvsp[0].assign));
	}
#line 1863 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 87: /* net_assignments: net_assignments ',' net_assignment  */
#line 348 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyvsp[-2].stmt_seq)->push_back((yyvsp[0].assign)); }
#line 1869 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 88: /* @6: %empty  */
#line 352 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
                       { (yyval.ival) = sta::verilog_reader->line(); }
#line 1875 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 89: /* net_assignment: net_assign_lhs @6 '=' net_expr  */
#line 353 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.assign) = sta::verilog_reader->makeAssign((yyvsp[-3].net), (yyvsp[0].net), (yyvsp[-2].ival)); }
#line 1881 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 92: /* @7: %empty  */
#line 362 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
                                  { (yyval.ival) = sta::verilog_reader->line(); }
#line 1887 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 93: /* instance: attribute_instance_seq ID @7 ID '(' inst_pins ')' ';'  */
#line 363 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.stmt) = sta::verilog_reader->makeModuleInst((yyvsp[-6].string), (yyvsp[-4].string), (yyvsp[-2].nets), (yyvsp[-7].attribute_stmt_seq), (yyvsp[-5].ival)); }
#line 1893 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 94: /* @8: %empty  */
#line 364 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
                                  { (yyval.ival) = sta::verilog_reader->line(); }
#line 1899 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 95: /* instance: attribute_instance_seq ID @8 parameter_values ID '(' inst_pins ')' ';'  */
#line 366 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.stmt) = sta::verilog_reader->makeModuleInst((yyvsp[-7].string), (yyvsp[-4].string), (yyvsp[-2].nets), (yyvsp[-8].attribute_stmt_seq), (yyvsp[-6].ival)); }
#line 1905 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 98: /* parameter_exprs: '{' parameter_exprs '}'  */
#line 376 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.ival) = (yyvsp[-1].ival); }
#line 1911 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 100: /* inst_pins: %empty  */
#line 382 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.nets) = NULL; }
#line 1917 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 103: /* inst_ordered_pins: net_expr  */
#line 390 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.nets) = new sta::VerilogNetSeq;
	  (yyval.nets)->push_back((yyvsp[0].net));
	}
#line 1925 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 104: /* inst_ordered_pins: inst_ordered_pins ',' net_expr  */
#line 394 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyvsp[-2].nets)->push_back((yyvsp[0].net)); }
#line 1931 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 105: /* inst_named_pins: inst_named_pin  */
#line 400 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.nets) = new sta::VerilogNetSeq;
	  (yyval.nets)->push_back((yyvsp[0].net));
	}
#line 1939 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 106: /* inst_named_pins: inst_named_pins ',' inst_named_pin  */
#line 404 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyvsp[-2].nets)->push_back((yyvsp[0].net)); }
#line 1945 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 107: /* inst_named_pin: '.' ID '(' ')'  */
#line 412 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.net) = sta::verilog_reader->makeNetNamedPortRefScalarNet((yyvsp[-2].string)); }
#line 1951 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 108: /* inst_named_pin: '.' ID '(' ID ')'  */
#line 414 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.net) = sta::verilog_reader->makeNetNamedPortRefScalarNet((yyvsp[-3].string), (yyvsp[-1].string)); }
#line 1957 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 109: /* inst_named_pin: '.' ID '(' ID '[' INT ']' ')'  */
#line 416 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.net) = sta::verilog_reader->makeNetNamedPortRefBitSelect((yyvsp[-6].string), (yyvsp[-4].string), (yyvsp[-2].ival)); }
#line 1963 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 110: /* inst_named_pin: '.' ID '(' named_pin_net_expr ')'  */
#line 418 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.net) = sta::verilog_reader->makeNetNamedPortRefScalar((yyvsp[-3].string), (yyvsp[-1].net)); }
#line 1969 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 111: /* inst_named_pin: '.' ID '[' INT ']' '(' ')'  */
#line 421 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.net) = sta::verilog_reader->makeNetNamedPortRefBit((yyvsp[-5].string), (yyvsp[-3].ival), NULL); }
#line 1975 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 112: /* inst_named_pin: '.' ID '[' INT ']' '(' net_expr ')'  */
#line 423 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.net) = sta::verilog_reader->makeNetNamedPortRefBit((yyvsp[-6].string), (yyvsp[-4].ival), (yyvsp[-1].net)); }
#line 1981 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 113: /* inst_named_pin: '.' ID '[' INT ':' INT ']' '(' ')'  */
#line 426 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.net) = sta::verilog_reader->makeNetNamedPortRefPart((yyvsp[-7].string), (yyvsp[-5].ival), (yyvsp[-3].ival), NULL); }
#line 1987 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 114: /* inst_named_pin: '.' ID '[' INT ':' INT ']' '(' net_expr ')'  */
#line 428 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.net) = sta::verilog_reader->makeNetNamedPortRefPart((yyvsp[-8].string), (yyvsp[-6].ival), (yyvsp[-4].ival), (yyvsp[-1].net)); }
#line 1993 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 121: /* net_scalar: ID  */
#line 445 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.net) = sta::verilog_reader->makeNetScalar((yyvsp[0].string)); }
#line 1999 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 122: /* net_bit_select: ID '[' INT ']'  */
#line 450 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.net) = sta::verilog_reader->makeNetBitSelect((yyvsp[-3].string), (yyvsp[-1].ival)); }
#line 2005 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 123: /* net_part_select: ID '[' INT ':' INT ']'  */
#line 455 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.net) = sta::verilog_reader->makeNetPartSelect((yyvsp[-5].string), (yyvsp[-3].ival), (yyvsp[-1].ival)); }
#line 2011 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 124: /* net_constant: CONSTANT  */
#line 460 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.net) = sta::verilog_reader->makeNetConstant((yyvsp[0].constant)); }
#line 2017 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 125: /* net_expr_concat: '{' net_exprs '}'  */
#line 465 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.net) = sta::verilog_reader->makeNetConcat((yyvsp[-1].nets)); }
#line 2023 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 126: /* net_exprs: net_expr  */
#line 470 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.nets) = new sta::VerilogNetSeq;
	  (yyval.nets)->push_back((yyvsp[0].net));
	}
#line 2031 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 127: /* net_exprs: net_exprs ',' net_expr  */
#line 474 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.nets)->push_back((yyvsp[0].net)); }
#line 2037 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 133: /* attribute_instance_seq: %empty  */
#line 487 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.attribute_stmt_seq) = new sta::VerilogAttributeStmtSeq; }
#line 2043 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 134: /* attribute_instance_seq: attribute_instance_seq attribute_instance  */
#line 489 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { if ((yyvsp[0].attribute_stmt)) (yyvsp[-1].attribute_stmt_seq)->push_back((yyvsp[0].attribute_stmt)); }
#line 2049 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 135: /* attribute_instance: ATTRIBUTE_OPEN attr_specs ATTRIBUTE_CLOSED  */
#line 494 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.attribute_stmt) = new sta::VerilogAttributeStmt((yyvsp[-1].attribute_seq)); }
#line 2055 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 136: /* attr_specs: attr_spec  */
#line 499 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.attribute_seq) = new sta::VerilogAttributeEntrySeq;
	  (yyval.attribute_seq)->push_back((yyvsp[0].attribute_entry));
	}
#line 2063 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 137: /* attr_specs: attr_specs ',' attr_spec  */
#line 503 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.attribute_seq)->push_back((yyvsp[0].attribute_entry)); }
#line 2069 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 138: /* attr_spec: ID  */
#line 508 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.attribute_entry) = new sta::VerilogAttributeEntry((yyvsp[0].string), "1");
	  delete[] (yyvsp[0].string);
	}
#line 2077 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 139: /* attr_spec: ID '=' attr_spec_value  */
#line 512 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.attribute_entry) = new sta::VerilogAttributeEntry((yyvsp[-2].string), (yyvsp[0].attribute_spec_value)); 
	  delete[] (yyvsp[-2].string);
	  delete[] (yyvsp[0].attribute_spec_value);
	}
#line 2086 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 140: /* attr_spec_value: CONSTANT  */
#line 520 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.attribute_spec_value) = (yyvsp[0].constant); }
#line 2092 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 141: /* attr_spec_value: STRING  */
#line 522 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.attribute_spec_value) = (yyvsp[0].string); }
#line 2098 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;

  case 142: /* attr_spec_value: INT  */
#line 524 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"
        { (yyval.attribute_spec_value) = sta::stringCopy(std::to_string((yyvsp[0].ival)).c_str()); }
#line 2104 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"
    break;


#line 2108 "/home/captainnothing/OpenROAD/build/src/sta/VerilogParse.cc"

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

#line 527 "/home/captainnothing/OpenROAD/src/sta/verilog/VerilogParse.yy"

