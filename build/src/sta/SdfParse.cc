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
#define yyparse         SdfParse_parse
#define yylex           SdfParse_lex
#define yyerror         SdfParse_error
#define yydebug         SdfParse_debug
#define yynerrs         SdfParse_nerrs
#define yylval          SdfParse_lval
#define yychar          SdfParse_char

/* First part of user prologue.  */
#line 1 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"


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

#include <cctype>

#include "sdf/SdfReaderPvt.hh"

int SdfLex_lex();
#define SdfParse_lex SdfLex_lex
// use yacc generated parser errors
#define YYERROR_VERBOSE

#define YYDEBUG 1


#line 109 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"

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

#include "SdfParse.hh"
/* Symbol kind.  */
enum yysymbol_kind_t
{
  YYSYMBOL_YYEMPTY = -2,
  YYSYMBOL_YYEOF = 0,                      /* "end of file"  */
  YYSYMBOL_YYerror = 1,                    /* error  */
  YYSYMBOL_YYUNDEF = 2,                    /* "invalid token"  */
  YYSYMBOL_DELAYFILE = 3,                  /* DELAYFILE  */
  YYSYMBOL_SDFVERSION = 4,                 /* SDFVERSION  */
  YYSYMBOL_DESIGN = 5,                     /* DESIGN  */
  YYSYMBOL_DATE = 6,                       /* DATE  */
  YYSYMBOL_VENDOR = 7,                     /* VENDOR  */
  YYSYMBOL_PROGRAM = 8,                    /* PROGRAM  */
  YYSYMBOL_PVERSION = 9,                   /* PVERSION  */
  YYSYMBOL_DIVIDER = 10,                   /* DIVIDER  */
  YYSYMBOL_VOLTAGE = 11,                   /* VOLTAGE  */
  YYSYMBOL_PROCESS = 12,                   /* PROCESS  */
  YYSYMBOL_TEMPERATURE = 13,               /* TEMPERATURE  */
  YYSYMBOL_TIMESCALE = 14,                 /* TIMESCALE  */
  YYSYMBOL_CELL = 15,                      /* CELL  */
  YYSYMBOL_CELLTYPE = 16,                  /* CELLTYPE  */
  YYSYMBOL_INSTANCE = 17,                  /* INSTANCE  */
  YYSYMBOL_DELAY = 18,                     /* DELAY  */
  YYSYMBOL_ABSOLUTE = 19,                  /* ABSOLUTE  */
  YYSYMBOL_INCREMENTAL = 20,               /* INCREMENTAL  */
  YYSYMBOL_INTERCONNECT = 21,              /* INTERCONNECT  */
  YYSYMBOL_PORT = 22,                      /* PORT  */
  YYSYMBOL_DEVICE = 23,                    /* DEVICE  */
  YYSYMBOL_RETAIN = 24,                    /* RETAIN  */
  YYSYMBOL_IOPATH = 25,                    /* IOPATH  */
  YYSYMBOL_TIMINGCHECK = 26,               /* TIMINGCHECK  */
  YYSYMBOL_SETUP = 27,                     /* SETUP  */
  YYSYMBOL_HOLD = 28,                      /* HOLD  */
  YYSYMBOL_SETUPHOLD = 29,                 /* SETUPHOLD  */
  YYSYMBOL_RECOVERY = 30,                  /* RECOVERY  */
  YYSYMBOL_REMOVAL = 31,                   /* REMOVAL  */
  YYSYMBOL_RECREM = 32,                    /* RECREM  */
  YYSYMBOL_WIDTH = 33,                     /* WIDTH  */
  YYSYMBOL_PERIOD = 34,                    /* PERIOD  */
  YYSYMBOL_SKEW = 35,                      /* SKEW  */
  YYSYMBOL_NOCHANGE = 36,                  /* NOCHANGE  */
  YYSYMBOL_POSEDGE = 37,                   /* POSEDGE  */
  YYSYMBOL_NEGEDGE = 38,                   /* NEGEDGE  */
  YYSYMBOL_COND = 39,                      /* COND  */
  YYSYMBOL_CONDELSE = 40,                  /* CONDELSE  */
  YYSYMBOL_QSTRING = 41,                   /* QSTRING  */
  YYSYMBOL_ID = 42,                        /* ID  */
  YYSYMBOL_FNUMBER = 43,                   /* FNUMBER  */
  YYSYMBOL_DNUMBER = 44,                   /* DNUMBER  */
  YYSYMBOL_EXPR_OPEN_IOPATH = 45,          /* EXPR_OPEN_IOPATH  */
  YYSYMBOL_EXPR_OPEN = 46,                 /* EXPR_OPEN  */
  YYSYMBOL_EXPR_ID_CLOSE = 47,             /* EXPR_ID_CLOSE  */
  YYSYMBOL_48_ = 48,                       /* '('  */
  YYSYMBOL_49_ = 49,                       /* ')'  */
  YYSYMBOL_50_ = 50,                       /* '/'  */
  YYSYMBOL_51_ = 51,                       /* '.'  */
  YYSYMBOL_52_ = 52,                       /* '*'  */
  YYSYMBOL_53_ = 53,                       /* '['  */
  YYSYMBOL_54_ = 54,                       /* ']'  */
  YYSYMBOL_55_ = 55,                       /* ':'  */
  YYSYMBOL_56_ = 56,                       /* '-'  */
  YYSYMBOL_YYACCEPT = 57,                  /* $accept  */
  YYSYMBOL_file = 58,                      /* file  */
  YYSYMBOL_header = 59,                    /* header  */
  YYSYMBOL_header_stmt = 60,               /* header_stmt  */
  YYSYMBOL_hchar = 61,                     /* hchar  */
  YYSYMBOL_number_opt = 62,                /* number_opt  */
  YYSYMBOL_cells = 63,                     /* cells  */
  YYSYMBOL_cell = 64,                      /* cell  */
  YYSYMBOL_celltype = 65,                  /* celltype  */
  YYSYMBOL_cell_instance = 66,             /* cell_instance  */
  YYSYMBOL_timing_specs = 67,              /* timing_specs  */
  YYSYMBOL_timing_spec = 68,               /* timing_spec  */
  YYSYMBOL_deltypes = 69,                  /* deltypes  */
  YYSYMBOL_deltype = 70,                   /* deltype  */
  YYSYMBOL_71_1 = 71,                      /* $@1  */
  YYSYMBOL_72_2 = 72,                      /* $@2  */
  YYSYMBOL_del_defs = 73,                  /* del_defs  */
  YYSYMBOL_path = 74,                      /* path  */
  YYSYMBOL_del_def = 75,                   /* del_def  */
  YYSYMBOL_retains = 76,                   /* retains  */
  YYSYMBOL_retain = 77,                    /* retain  */
  YYSYMBOL_delval_list = 78,               /* delval_list  */
  YYSYMBOL_tchk_defs = 79,                 /* tchk_defs  */
  YYSYMBOL_tchk_def = 80,                  /* tchk_def  */
  YYSYMBOL_81_3 = 81,                      /* $@3  */
  YYSYMBOL_82_4 = 82,                      /* $@4  */
  YYSYMBOL_83_5 = 83,                      /* $@5  */
  YYSYMBOL_84_6 = 84,                      /* $@6  */
  YYSYMBOL_85_7 = 85,                      /* $@7  */
  YYSYMBOL_86_8 = 86,                      /* $@8  */
  YYSYMBOL_87_9 = 87,                      /* $@9  */
  YYSYMBOL_88_10 = 88,                     /* $@10  */
  YYSYMBOL_89_11 = 89,                     /* $@11  */
  YYSYMBOL_90_12 = 90,                     /* $@12  */
  YYSYMBOL_port = 91,                      /* port  */
  YYSYMBOL_port_instance = 92,             /* port_instance  */
  YYSYMBOL_port_spec = 93,                 /* port_spec  */
  YYSYMBOL_port_transition = 94,           /* port_transition  */
  YYSYMBOL_port_tchk = 95,                 /* port_tchk  */
  YYSYMBOL_value = 96,                     /* value  */
  YYSYMBOL_triple = 97,                    /* triple  */
  YYSYMBOL_NUMBER = 98                     /* NUMBER  */
};
typedef enum yysymbol_kind_t yysymbol_kind_t;


/* Second part of user prologue.  */
#line 69 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"


#line 244 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"


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
typedef yytype_uint8 yy_state_t;

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
#define YYFINAL  4
/* YYLAST -- Last index in YYTABLE.  */
#define YYLAST   270

/* YYNTOKENS -- Number of terminals.  */
#define YYNTOKENS  57
/* YYNNTS -- Number of nonterminals.  */
#define YYNNTS  42
/* YYNRULES -- Number of rules.  */
#define YYNRULES  99
/* YYNSTATES -- Number of states.  */
#define YYNSTATES  253

/* YYMAXUTOK -- Last valid token kind.  */
#define YYMAXUTOK   302


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
      48,    49,    52,     2,     2,    56,    51,    50,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,    55,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,    53,     2,    54,     2,     2,     2,     2,     2,     2,
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
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     1,     2,     3,     4,
       5,     6,     7,     8,     9,    10,    11,    12,    13,    14,
      15,    16,    17,    18,    19,    20,    21,    22,    23,    24,
      25,    26,    27,    28,    29,    30,    31,    32,    33,    34,
      35,    36,    37,    38,    39,    40,    41,    42,    43,    44,
      45,    46,    47
};

#if YYDEBUG
/* YYRLINE[YYN] -- Source line where rule number YYN was defined.  */
static const yytype_int16 yyrline[] =
{
       0,    75,    75,    79,    80,    85,    86,    87,    88,    89,
      90,    91,    92,    93,    94,    95,    96,    97,    98,    99,
     100,   105,   107,   111,   112,   116,   117,   121,   126,   131,
     133,   135,   139,   141,   145,   146,   149,   150,   155,   154,
     158,   157,   162,   163,   167,   169,   174,   176,   179,   182,
     184,   186,   188,   192,   194,   198,   203,   205,   209,   210,
     214,   214,   219,   219,   224,   224,   229,   229,   234,   234,
     239,   239,   244,   244,   250,   250,   255,   255,   260,   260,
     268,   270,   278,   279,   284,   286,   291,   292,   296,   297,
     299,   304,   308,   312,   316,   321,   326,   334,   335,   337
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
  "\"end of file\"", "error", "\"invalid token\"", "DELAYFILE",
  "SDFVERSION", "DESIGN", "DATE", "VENDOR", "PROGRAM", "PVERSION",
  "DIVIDER", "VOLTAGE", "PROCESS", "TEMPERATURE", "TIMESCALE", "CELL",
  "CELLTYPE", "INSTANCE", "DELAY", "ABSOLUTE", "INCREMENTAL",
  "INTERCONNECT", "PORT", "DEVICE", "RETAIN", "IOPATH", "TIMINGCHECK",
  "SETUP", "HOLD", "SETUPHOLD", "RECOVERY", "REMOVAL", "RECREM", "WIDTH",
  "PERIOD", "SKEW", "NOCHANGE", "POSEDGE", "NEGEDGE", "COND", "CONDELSE",
  "QSTRING", "ID", "FNUMBER", "DNUMBER", "EXPR_OPEN_IOPATH", "EXPR_OPEN",
  "EXPR_ID_CLOSE", "'('", "')'", "'/'", "'.'", "'*'", "'['", "']'", "':'",
  "'-'", "$accept", "file", "header", "header_stmt", "hchar", "number_opt",
  "cells", "cell", "celltype", "cell_instance", "timing_specs",
  "timing_spec", "deltypes", "deltype", "$@1", "$@2", "del_defs", "path",
  "del_def", "retains", "retain", "delval_list", "tchk_defs", "tchk_def",
  "$@3", "$@4", "$@5", "$@6", "$@7", "$@8", "$@9", "$@10", "$@11", "$@12",
  "port", "port_instance", "port_spec", "port_transition", "port_tchk",
  "value", "triple", "NUMBER", YY_NULLPTR
};

static const char *
yysymbol_name (yysymbol_kind_t yysymbol)
{
  return yytname[yysymbol];
}
#endif

#define YYPACT_NINF (-188)

#define yypact_value_is_default(Yyn) \
  ((Yyn) == YYPACT_NINF)

#define YYTABLE_NINF (-46)

#define yytable_value_is_error(Yyn) \
  0

/* YYPACT[STATE-NUM] -- Index in YYTABLE of the portion describing
   STATE-NUM.  */
static const yytype_int16 yypact[] =
{
     -11,    44,    61,    16,  -188,   146,    18,  -188,    38,    40,
      64,    66,    77,    82,   -17,   -32,    -9,   -18,    76,   132,
    -188,   133,  -188,    84,    99,   124,   166,   167,   168,  -188,
    -188,   169,  -188,  -188,  -188,    49,    70,   171,   -19,   173,
    -188,  -188,   174,    34,   177,   176,   210,  -188,  -188,  -188,
    -188,  -188,  -188,  -188,  -188,  -188,  -188,    76,  -188,  -188,
      76,  -188,  -188,  -188,   178,   212,   181,   175,   179,   180,
    -188,  -188,   185,   214,  -188,    76,    76,    76,   183,    -1,
     135,  -188,  -188,  -188,  -188,  -188,  -188,   184,   126,    36,
    -188,  -188,  -188,  -188,   194,  -188,  -188,  -188,   137,   139,
     170,  -188,  -188,   134,  -188,  -188,  -188,  -188,  -188,  -188,
    -188,  -188,  -188,  -188,  -188,  -188,  -188,  -188,  -188,  -188,
      52,    52,    52,    52,    52,    52,    52,    52,    52,    52,
     143,   145,    71,   141,   -17,  -188,  -188,  -188,    52,    52,
      52,    52,    52,    52,   189,   189,    52,    52,    51,  -188,
    -188,  -188,   195,  -188,  -188,   149,   196,   198,   189,   189,
     189,   189,   189,   189,    43,   192,   193,   189,   189,   196,
     196,    53,    54,   199,   197,   200,   160,  -188,   201,   121,
    -188,   202,   203,   189,   204,   206,   189,  -188,   207,    48,
    -188,  -188,   208,   189,   196,   189,   151,   189,  -188,   160,
     196,    54,   218,  -188,   196,  -188,  -188,  -188,   209,  -188,
    -188,   211,  -188,  -188,  -188,   213,   189,   153,  -188,  -188,
     155,  -188,   196,    54,   215,  -188,  -188,  -188,   157,  -188,
    -188,   217,  -188,   196,   219,  -188,    26,  -188,   159,   217,
    -188,  -188,   189,  -188,   161,   217,   163,   220,   165,  -188,
    -188,   221,  -188
};

/* YYDEFACT[STATE-NUM] -- Default reduction number in state STATE-NUM.
   Performed when YYTABLE does not specify something else to do.  Zero
   means the default is an error.  */
static const yytype_int8 yydefact[] =
{
       0,     0,     0,     0,     1,     0,     0,     3,     0,     0,
       0,     0,     0,     0,     0,    23,     0,    23,     0,     0,
       4,     0,    25,     0,     0,     0,     0,     0,     0,    21,
      22,     0,    97,    98,    14,     0,     0,     0,     0,     0,
      16,    19,     0,     0,     0,     0,     0,     2,    26,     5,
       6,     7,     8,     9,    10,    11,    99,    23,    12,    13,
      23,    15,    18,    17,     0,     0,     0,     0,     0,     0,
      24,    20,     0,     0,    32,     0,    23,    23,     0,     0,
       0,    96,    95,    94,    28,    44,    29,     0,     0,     0,
      27,    33,    30,    31,     0,    36,    58,    45,     0,     0,
       0,    34,    37,     0,    35,    59,    38,    40,    60,    62,
      64,    66,    68,    70,    74,    76,    72,    78,    42,    42,
       0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       0,     0,    80,     0,     0,    82,    84,    88,     0,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     0,    39,
      43,    41,     0,    86,    87,     0,     0,     0,     0,     0,
       0,     0,     0,     0,    23,     0,     0,     0,     0,     0,
       0,     0,     0,     0,     0,     0,     0,    89,     0,    80,
      83,     0,     0,     0,     0,     0,     0,    91,     0,     0,
      75,    77,     0,     0,     0,     0,     0,     0,    56,     0,
       0,     0,     0,    81,     0,    85,    61,    63,     0,    67,
      69,     0,    93,    92,    73,     0,     0,     0,    51,    57,
       0,    53,     0,     0,     0,    65,    71,    79,     0,    50,
      52,     0,    53,     0,     0,    49,    23,    54,     0,     0,
      53,    90,     0,    46,     0,     0,     0,     0,     0,    55,
      48,     0,    47
};

/* YYPGOTO[NTERM-NUM].  */
static const yytype_int16 yypgoto[] =
{
    -188,  -188,  -188,   240,   -75,     8,  -188,   226,  -188,  -188,
    -188,  -188,  -188,  -188,  -188,  -188,   129,   182,  -188,  -169,
    -188,  -187,  -188,  -188,  -188,  -188,  -188,  -188,  -188,  -188,
    -188,  -188,  -188,  -188,    92,  -155,  -166,    83,   -12,  -140,
      -8,   -15
};

/* YYDEFGOTO[NTERM-NUM].  */
static const yytype_uint8 yydefgoto[] =
{
       0,     2,     6,     7,    31,    36,    21,    22,    66,    74,
      80,    91,    98,   102,   118,   119,   130,   134,   150,   231,
     237,   196,    99,   105,   120,   121,   122,   123,   124,   125,
     128,   126,   127,   129,   135,   136,   137,   156,   138,   198,
     188,    70
};

/* YYTABLE[YYPACT[STATE-NUM]] -- What to do in state STATE-NUM.  If
   positive, shift that token.  If negative, reduce the rule whose
   number is the opposite.  If YYTABLE_NINF, syntax error.  */
static const yytype_int16 yytable[] =
{
      38,   178,    43,    44,   165,   166,   200,    37,   217,    42,
     220,    32,    33,    94,   194,   195,   197,    34,   181,   182,
     183,   184,   185,   186,    35,    32,    33,   192,   193,   228,
      59,    41,    39,    29,    30,   222,    60,     1,    35,   216,
      40,    85,    68,   208,   238,   221,   211,     3,    86,   224,
     242,    87,   244,   215,    95,   246,   219,   233,   248,   157,
      81,     4,    96,   239,     5,    67,    19,   232,    69,    32,
      33,   245,   169,   170,   171,   187,   172,   219,   240,    23,
     219,    24,    35,    63,    82,    83,    32,    33,   219,    60,
     173,   174,   187,    56,   132,   132,   132,   213,   219,    35,
     133,   164,   199,    60,   219,    25,   219,    26,   219,   139,
     140,   141,   142,   143,   144,   145,   146,   147,    27,    32,
      33,   -44,   -44,    28,   152,    57,   158,   159,   160,   161,
     162,   163,    35,    49,   167,   168,     8,     9,    10,    11,
      12,    13,    14,    15,    16,    17,    18,    45,    50,   189,
       8,     9,    10,    11,    12,    13,    14,    15,    16,    17,
      18,   108,   109,   110,   111,   112,   113,   114,   115,   116,
     117,   -45,   -45,    51,   152,    93,    29,    30,   153,   154,
     155,    46,    47,    89,    90,   100,   101,   103,   104,   106,
     107,   148,   149,   148,   151,   176,   177,   153,   154,   164,
     218,   164,   229,   164,   230,   164,   235,   164,   243,   164,
     247,   164,   249,   164,   251,    52,    53,    54,    55,    64,
      58,   189,    61,    62,    65,    45,    78,    71,    72,    73,
      75,    79,    84,    92,    76,    77,    97,   164,   132,   175,
     179,   190,   191,   223,   201,   202,    20,    48,   131,   180,
     205,   206,   207,   209,   203,   210,   212,   214,   225,   204,
     226,    88,   227,     0,   234,   236,     0,     0,   241,   250,
     252
};

static const yytype_int16 yycheck[] =
{
      15,   156,    17,    18,   144,   145,   172,    15,   195,    17,
     197,    43,    44,    88,   169,   170,   171,    49,   158,   159,
     160,   161,   162,   163,    56,    43,    44,   167,   168,   216,
      49,    49,    41,    50,    51,   201,    55,    48,    56,   194,
      49,    42,    57,   183,   231,   200,   186,     3,    49,   204,
      24,    52,   239,   193,    18,   242,   196,   223,   245,   134,
      75,     0,    26,   232,    48,    57,    48,   222,    60,    43,
      44,   240,    21,    22,    23,    49,    25,   217,   233,    41,
     220,    41,    56,    49,    76,    77,    43,    44,   228,    55,
      39,    40,    49,    44,    42,    42,    42,    49,   238,    56,
      48,    48,    48,    55,   244,    41,   246,    41,   248,   121,
     122,   123,   124,   125,   126,   127,   128,   129,    41,    43,
      44,    50,    51,    41,    53,    55,   138,   139,   140,   141,
     142,   143,    56,    49,   146,   147,     4,     5,     6,     7,
       8,     9,    10,    11,    12,    13,    14,    15,    49,   164,
       4,     5,     6,     7,     8,     9,    10,    11,    12,    13,
      14,    27,    28,    29,    30,    31,    32,    33,    34,    35,
      36,    50,    51,    49,    53,    49,    50,    51,    37,    38,
      39,    48,    49,    48,    49,    48,    49,    48,    49,    19,
      20,    48,    49,    48,    49,    46,    47,    37,    38,    48,
      49,    48,    49,    48,    49,    48,    49,    48,    49,    48,
      49,    48,    49,    48,    49,    49,    49,    49,    49,    42,
      49,   236,    49,    49,    48,    15,    41,    49,    16,    48,
      55,    17,    49,    49,    55,    55,    42,    48,    42,    44,
      42,    49,    49,    25,    45,    48,     6,    21,   119,   157,
      49,    49,    49,    49,    54,    49,    49,    49,    49,   176,
      49,    79,    49,    -1,    49,    48,    -1,    -1,    49,    49,
      49
};

/* YYSTOS[STATE-NUM] -- The symbol kind of the accessing symbol of
   state STATE-NUM.  */
static const yytype_int8 yystos[] =
{
       0,    48,    58,     3,     0,    48,    59,    60,     4,     5,
       6,     7,     8,     9,    10,    11,    12,    13,    14,    48,
      60,    63,    64,    41,    41,    41,    41,    41,    41,    50,
      51,    61,    43,    44,    49,    56,    62,    97,    98,    41,
      49,    49,    97,    98,    98,    15,    48,    49,    64,    49,
      49,    49,    49,    49,    49,    49,    44,    55,    49,    49,
      55,    49,    49,    49,    42,    48,    65,    62,    98,    62,
      98,    49,    16,    48,    66,    55,    55,    55,    41,    17,
      67,    98,    62,    62,    49,    42,    49,    52,    74,    48,
      49,    68,    49,    49,    61,    18,    26,    42,    69,    79,
      48,    49,    70,    48,    49,    80,    19,    20,    27,    28,
      29,    30,    31,    32,    33,    34,    35,    36,    71,    72,
      81,    82,    83,    84,    85,    86,    88,    89,    87,    90,
      73,    73,    42,    48,    74,    91,    92,    93,    95,    95,
      95,    95,    95,    95,    95,    95,    95,    95,    48,    49,
      75,    49,    53,    37,    38,    39,    94,    61,    95,    95,
      95,    95,    95,    95,    48,    96,    96,    95,    95,    21,
      22,    23,    25,    39,    40,    44,    46,    47,    92,    42,
      91,    96,    96,    96,    96,    96,    96,    49,    97,    98,
      49,    49,    96,    96,    92,    92,    78,    92,    96,    48,
      93,    45,    48,    54,    94,    49,    49,    49,    96,    49,
      49,    96,    49,    49,    49,    96,    92,    78,    49,    96,
      78,    92,    93,    25,    92,    49,    49,    49,    78,    49,
      49,    76,    92,    93,    49,    49,    48,    77,    78,    76,
      92,    49,    24,    49,    78,    76,    78,    49,    78,    49,
      49,    49,    49
};

/* YYR1[RULE-NUM] -- Symbol kind of the left-hand side of rule RULE-NUM.  */
static const yytype_int8 yyr1[] =
{
       0,    57,    58,    59,    59,    60,    60,    60,    60,    60,
      60,    60,    60,    60,    60,    60,    60,    60,    60,    60,
      60,    61,    61,    62,    62,    63,    63,    64,    65,    66,
      66,    66,    67,    67,    68,    68,    69,    69,    71,    70,
      72,    70,    73,    73,    74,    74,    75,    75,    75,    75,
      75,    75,    75,    76,    76,    77,    78,    78,    79,    79,
      81,    80,    82,    80,    83,    80,    84,    80,    85,    80,
      86,    80,    87,    80,    88,    80,    89,    80,    90,    80,
      91,    91,    92,    92,    93,    93,    94,    94,    95,    95,
      95,    96,    96,    96,    97,    97,    97,    98,    98,    98
};

/* YYR2[RULE-NUM] -- Number of symbols on the right-hand side of rule RULE-NUM.  */
static const yytype_int8 yyr2[] =
{
       0,     2,     5,     1,     2,     4,     4,     4,     4,     4,
       4,     4,     4,     4,     3,     4,     3,     4,     4,     3,
       5,     1,     1,     0,     1,     1,     2,     6,     4,     3,
       4,     4,     0,     2,     4,     4,     0,     2,     0,     5,
       0,     5,     0,     2,     1,     3,     7,    10,     9,     6,
       5,     4,     5,     0,     2,     4,     1,     2,     0,     2,
       0,     7,     0,     7,     0,     8,     0,     7,     0,     7,
       0,     8,     0,     7,     0,     6,     0,     6,     0,     8,
       1,     4,     1,     3,     1,     4,     1,     1,     1,     3,
       7,     2,     3,     3,     5,     5,     5,     1,     1,     2
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
  case 2: /* file: '(' DELAYFILE header cells ')'  */
#line 75 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
                                        {}
#line 1368 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 5: /* header_stmt: '(' SDFVERSION QSTRING ')'  */
#line 85 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
                                   { sta::stringDelete((yyvsp[-1].string)); }
#line 1374 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 6: /* header_stmt: '(' DESIGN QSTRING ')'  */
#line 86 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
                               { sta::stringDelete((yyvsp[-1].string)); }
#line 1380 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 7: /* header_stmt: '(' DATE QSTRING ')'  */
#line 87 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
                             { sta::stringDelete((yyvsp[-1].string)); }
#line 1386 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 8: /* header_stmt: '(' VENDOR QSTRING ')'  */
#line 88 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
                               { sta::stringDelete((yyvsp[-1].string)); }
#line 1392 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 9: /* header_stmt: '(' PROGRAM QSTRING ')'  */
#line 89 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
                                { sta::stringDelete((yyvsp[-1].string)); }
#line 1398 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 10: /* header_stmt: '(' PVERSION QSTRING ')'  */
#line 90 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
                                 { sta::stringDelete((yyvsp[-1].string)); }
#line 1404 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 11: /* header_stmt: '(' DIVIDER hchar ')'  */
#line 91 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
                              { sta::sdf_reader->setDivider((yyvsp[-1].character)); }
#line 1410 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 12: /* header_stmt: '(' VOLTAGE triple ')'  */
#line 92 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
                               { sta::sdf_reader->deleteTriple((yyvsp[-1].triple)); }
#line 1416 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 15: /* header_stmt: '(' PROCESS QSTRING ')'  */
#line 95 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
                                { sta::stringDelete((yyvsp[-1].string)); }
#line 1422 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 18: /* header_stmt: '(' TEMPERATURE triple ')'  */
#line 98 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
                                   { sta::sdf_reader->deleteTriple((yyvsp[-1].triple)); }
#line 1428 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 20: /* header_stmt: '(' TIMESCALE NUMBER ID ')'  */
#line 101 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->setTimescale((yyvsp[-2].number), (yyvsp[-1].string)); }
#line 1434 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 21: /* hchar: '/'  */
#line 106 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { (yyval.character) = '/'; }
#line 1440 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 22: /* hchar: '.'  */
#line 108 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { (yyval.character) = '.'; }
#line 1446 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 23: /* number_opt: %empty  */
#line 111 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
            { (yyval.number_ptr) = NULL; }
#line 1452 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 24: /* number_opt: NUMBER  */
#line 112 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
                { (yyval.number_ptr) = new float((yyvsp[0].number)); }
#line 1458 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 27: /* cell: '(' CELL celltype cell_instance timing_specs ')'  */
#line 122 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->cellFinish(); }
#line 1464 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 28: /* celltype: '(' CELLTYPE QSTRING ')'  */
#line 127 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->setCell((yyvsp[-1].string)); }
#line 1470 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 29: /* cell_instance: '(' INSTANCE ')'  */
#line 132 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->setInstance(NULL); }
#line 1476 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 30: /* cell_instance: '(' INSTANCE '*' ')'  */
#line 134 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->setInstanceWildcard(); }
#line 1482 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 31: /* cell_instance: '(' INSTANCE path ')'  */
#line 136 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->setInstance((yyvsp[-1].string)); }
#line 1488 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 38: /* $@1: %empty  */
#line 155 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->setInIncremental(false); }
#line 1494 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 40: /* $@2: %empty  */
#line 158 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->setInIncremental(true); }
#line 1500 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 44: /* path: ID  */
#line 168 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { (yyval.string) = sta::sdf_reader->unescaped((yyvsp[0].string)); }
#line 1506 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 45: /* path: path hchar ID  */
#line 170 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { (yyval.string) = sta::sdf_reader->makePath((yyvsp[-2].string), sta::sdf_reader->unescaped((yyvsp[0].string))); }
#line 1512 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 46: /* del_def: '(' IOPATH port_spec port_instance retains delval_list ')'  */
#line 175 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->iopath((yyvsp[-4].port_spec), (yyvsp[-3].string), (yyvsp[-1].delval_list), NULL, false); }
#line 1518 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 47: /* del_def: '(' CONDELSE '(' IOPATH port_spec port_instance retains delval_list ')' ')'  */
#line 178 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->iopath((yyvsp[-5].port_spec), (yyvsp[-4].string), (yyvsp[-2].delval_list), NULL, true); }
#line 1524 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 48: /* del_def: '(' COND EXPR_OPEN_IOPATH port_spec port_instance retains delval_list ')' ')'  */
#line 181 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->iopath((yyvsp[-5].port_spec), (yyvsp[-4].string), (yyvsp[-2].delval_list), (yyvsp[-6].string), false); }
#line 1530 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 49: /* del_def: '(' INTERCONNECT port_instance port_instance delval_list ')'  */
#line 183 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->interconnect((yyvsp[-3].string), (yyvsp[-2].string), (yyvsp[-1].delval_list)); }
#line 1536 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 50: /* del_def: '(' PORT port_instance delval_list ')'  */
#line 185 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->port((yyvsp[-2].string), (yyvsp[-1].delval_list)); }
#line 1542 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 51: /* del_def: '(' DEVICE delval_list ')'  */
#line 187 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->device((yyvsp[-1].delval_list)); }
#line 1548 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 52: /* del_def: '(' DEVICE port_instance delval_list ')'  */
#line 189 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->device((yyvsp[-2].string), (yyvsp[-1].delval_list)); }
#line 1554 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 55: /* retain: '(' RETAIN delval_list ')'  */
#line 199 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->deleteTripleSeq((yyvsp[-1].delval_list)); }
#line 1560 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 56: /* delval_list: value  */
#line 204 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { (yyval.delval_list) = sta::sdf_reader->makeTripleSeq(); (yyval.delval_list)->push_back((yyvsp[0].triple)); }
#line 1566 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 57: /* delval_list: delval_list value  */
#line 206 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { (yyvsp[-1].delval_list)->push_back((yyvsp[0].triple)); (yyval.delval_list) = (yyvsp[-1].delval_list); }
#line 1572 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 60: /* $@3: %empty  */
#line 214 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
                  { sta::sdf_reader->setInTimingCheck(true); }
#line 1578 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 61: /* tchk_def: '(' SETUP $@3 port_tchk port_tchk value ')'  */
#line 216 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->timingCheck(sta::TimingRole::setup(), (yyvsp[-3].port_spec), (yyvsp[-2].port_spec), (yyvsp[-1].triple));
	  sta::sdf_reader->setInTimingCheck(false);
	}
#line 1586 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 62: /* $@4: %empty  */
#line 219 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
                 { sta::sdf_reader->setInTimingCheck(true); }
#line 1592 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 63: /* tchk_def: '(' HOLD $@4 port_tchk port_tchk value ')'  */
#line 221 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->timingCheck(sta::TimingRole::hold(), (yyvsp[-3].port_spec), (yyvsp[-2].port_spec), (yyvsp[-1].triple));
	  sta::sdf_reader->setInTimingCheck(false);
	}
#line 1600 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 64: /* $@5: %empty  */
#line 224 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
                      { sta::sdf_reader->setInTimingCheck(true); }
#line 1606 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 65: /* tchk_def: '(' SETUPHOLD $@5 port_tchk port_tchk value value ')'  */
#line 226 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->timingCheckSetupHold((yyvsp[-4].port_spec), (yyvsp[-3].port_spec), (yyvsp[-2].triple), (yyvsp[-1].triple));
	  sta::sdf_reader->setInTimingCheck(false);
	}
#line 1614 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 66: /* $@6: %empty  */
#line 229 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
                     { sta::sdf_reader->setInTimingCheck(true); }
#line 1620 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 67: /* tchk_def: '(' RECOVERY $@6 port_tchk port_tchk value ')'  */
#line 231 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->timingCheck(sta::TimingRole::recovery(),(yyvsp[-3].port_spec),(yyvsp[-2].port_spec),(yyvsp[-1].triple));
	  sta::sdf_reader->setInTimingCheck(false);
	}
#line 1628 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 68: /* $@7: %empty  */
#line 234 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
                    { sta::sdf_reader->setInTimingCheck(true); }
#line 1634 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 69: /* tchk_def: '(' REMOVAL $@7 port_tchk port_tchk value ')'  */
#line 236 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->timingCheck(sta::TimingRole::removal(),(yyvsp[-3].port_spec),(yyvsp[-2].port_spec),(yyvsp[-1].triple));
	  sta::sdf_reader->setInTimingCheck(false);
	}
#line 1642 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 70: /* $@8: %empty  */
#line 239 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
                   { sta::sdf_reader->setInTimingCheck(true); }
#line 1648 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 71: /* tchk_def: '(' RECREM $@8 port_tchk port_tchk value value ')'  */
#line 241 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->timingCheckRecRem((yyvsp[-4].port_spec), (yyvsp[-3].port_spec), (yyvsp[-2].triple), (yyvsp[-1].triple));
	  sta::sdf_reader->setInTimingCheck(false);
	}
#line 1656 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 72: /* $@9: %empty  */
#line 244 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
                 { sta::sdf_reader->setInTimingCheck(true); }
#line 1662 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 73: /* tchk_def: '(' SKEW $@9 port_tchk port_tchk value ')'  */
#line 247 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->timingCheck(sta::TimingRole::skew(),(yyvsp[-2].port_spec),(yyvsp[-3].port_spec),(yyvsp[-1].triple));
	  sta::sdf_reader->setInTimingCheck(false);
	}
#line 1670 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 74: /* $@10: %empty  */
#line 250 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
                  { sta::sdf_reader->setInTimingCheck(true); }
#line 1676 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 75: /* tchk_def: '(' WIDTH $@10 port_tchk value ')'  */
#line 252 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->timingCheckWidth((yyvsp[-2].port_spec), (yyvsp[-1].triple));
	  sta::sdf_reader->setInTimingCheck(false);
	}
#line 1684 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 76: /* $@11: %empty  */
#line 255 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
                   { sta::sdf_reader->setInTimingCheck(true); }
#line 1690 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 77: /* tchk_def: '(' PERIOD $@11 port_tchk value ')'  */
#line 257 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->timingCheckPeriod((yyvsp[-2].port_spec), (yyvsp[-1].triple));
	  sta::sdf_reader->setInTimingCheck(false);
	}
#line 1698 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 78: /* $@12: %empty  */
#line 260 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
                     { sta::sdf_reader->setInTimingCheck(true); }
#line 1704 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 79: /* tchk_def: '(' NOCHANGE $@12 port_tchk port_tchk value value ')'  */
#line 262 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { sta::sdf_reader->timingCheckNochange((yyvsp[-4].port_spec), (yyvsp[-3].port_spec), (yyvsp[-2].triple), (yyvsp[-1].triple));
	  sta::sdf_reader->setInTimingCheck(false);
	}
#line 1712 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 80: /* port: ID  */
#line 269 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { (yyval.string) = sta::sdf_reader->unescaped((yyvsp[0].string)); }
#line 1718 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 81: /* port: ID '[' DNUMBER ']'  */
#line 271 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { const char *bus_name = sta::sdf_reader->unescaped((yyvsp[-3].string));
          (yyval.string) = sta::stringPrint("%s[%d]", bus_name, (yyvsp[-1].integer));
          sta::stringDelete(bus_name);
        }
#line 1727 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 83: /* port_instance: path hchar port  */
#line 280 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { (yyval.string) = sta::sdf_reader->makePath((yyvsp[-2].string), (yyvsp[0].string)); }
#line 1733 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 84: /* port_spec: port_instance  */
#line 285 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { (yyval.port_spec)=sta::sdf_reader->makePortSpec(sta::Transition::riseFall(),(yyvsp[0].string),NULL); }
#line 1739 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 85: /* port_spec: '(' port_transition port_instance ')'  */
#line 287 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { (yyval.port_spec) = sta::sdf_reader->makePortSpec((yyvsp[-2].transition), (yyvsp[-1].string), NULL); }
#line 1745 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 86: /* port_transition: POSEDGE  */
#line 291 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
                { (yyval.transition) = sta::Transition::rise(); }
#line 1751 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 87: /* port_transition: NEGEDGE  */
#line 292 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
                { (yyval.transition) = sta::Transition::fall(); }
#line 1757 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 89: /* port_tchk: '(' COND EXPR_ID_CLOSE  */
#line 298 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { (yyval.port_spec) = sta::sdf_reader->makeCondPortSpec((yyvsp[0].string)); }
#line 1763 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 90: /* port_tchk: '(' COND EXPR_OPEN port_transition port_instance ')' ')'  */
#line 300 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { (yyval.port_spec) = sta::sdf_reader->makePortSpec((yyvsp[-3].transition), (yyvsp[-2].string), (yyvsp[-4].string)); }
#line 1769 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 91: /* value: '(' ')'  */
#line 305 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        {
	  (yyval.triple) = sta::sdf_reader->makeTriple();
	}
#line 1777 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 92: /* value: '(' NUMBER ')'  */
#line 309 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        {
	  (yyval.triple) = sta::sdf_reader->makeTriple((yyvsp[-1].number));
	}
#line 1785 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 93: /* value: '(' triple ')'  */
#line 312 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
                       { (yyval.triple) = (yyvsp[-1].triple); }
#line 1791 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 94: /* triple: NUMBER ':' number_opt ':' number_opt  */
#line 317 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        {
	  float *fp = new float((yyvsp[-4].number));
	  (yyval.triple) = sta::sdf_reader->makeTriple(fp, (yyvsp[-2].number_ptr), (yyvsp[0].number_ptr));
	}
#line 1800 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 95: /* triple: number_opt ':' NUMBER ':' number_opt  */
#line 322 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        {
	  float *fp = new float((yyvsp[-2].number));
	  (yyval.triple) = sta::sdf_reader->makeTriple((yyvsp[-4].number_ptr), fp, (yyvsp[0].number_ptr));
	}
#line 1809 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 96: /* triple: number_opt ':' number_opt ':' NUMBER  */
#line 327 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        {
	  float *fp = new float((yyvsp[0].number));
	  (yyval.triple) = sta::sdf_reader->makeTriple((yyvsp[-4].number_ptr), (yyvsp[-2].number_ptr), fp);
	}
#line 1818 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 98: /* NUMBER: DNUMBER  */
#line 336 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { (yyval.number) = static_cast<float>((yyvsp[0].integer)); }
#line 1824 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;

  case 99: /* NUMBER: '-' DNUMBER  */
#line 338 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"
        { (yyval.number) = static_cast<float>(-(yyvsp[0].integer)); }
#line 1830 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"
    break;


#line 1834 "/home/captainnothing/OpenROAD/build/src/sta/SdfParse.cc"

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

#line 341 "/home/captainnothing/OpenROAD/src/sta/sdf/SdfParse.yy"

