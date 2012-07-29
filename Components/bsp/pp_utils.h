#ifndef pp_utils_h
#define pp_utils_h

/******************************************************************************/
/* General utility macros                                                     */
/******************************************************************************/
// the following macro will stringize the passed parameter
// Example: STRINGIZE( hello ) ==> "hello"
#define STRINGIZE( s ) STRINGIZE_X( s )
#define STRINGIZE_X( s ) #s

// the following macro will concatenate to parameters into a single parameter
// or identifier
// Example: CONCATENATE( GOOD, BYE ) ==> GOODBYE
#define CONCATENATE( a, b ) CONCATENATE_X( a, b )
#define CONCATENATE_X( a, b ) a ## b

// the following macro will concatenate two items infixed with the middle item
// Eample: INFIX( ANOTHER, _, TOKEN ) ==> ANOTHER_TOKEN
#define INFIX( a, i, b ) CONCATENATE( CONCATENATE( a, i ), b )


/******************************************************************************/
/* macros for declaring bit field uinons                                      */
/******************************************************************************/
// the following macro creates a union of with an encompasing group variable
// and a structure of bit fields.  the flds parameter most likely will be
// a DECL_BIT_FIELDS_x macro or a DECL_WIDTH_BIT_FIELDS_x macro or a semicolon
// delimited list of any combination of the two
#define DECL_BIT_FIELD_UNION( u_tag, u_name, grp_type, grp_name, flds_name, ... ) \
  union u_tag { grp_type grp_name; struct { __VA_ARGS__; } flds_name; } u_name

// the following macros define forms for declaring bit fields of a common type
// and common width
// Example: DECL_WIDTH_BIT_FIELDS_3( unsigned char, 2, a, , b );
// effectively yeilds...
//   unsigned char a : 2;
//   unsigned char   : 2;
//   unsigned char b : 2;
// NOTE: the second field has no name and is therefore a reserved field
#define  DECL_WIDTH_BIT_FIELDS_1( t, w, f )     						 DECL_BIT_FIELDS_1( t, f, w )
#define  DECL_WIDTH_BIT_FIELDS_2( t, w, f, ... )  DECL_WIDTH_BIT_FIELDS_1( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define  DECL_WIDTH_BIT_FIELDS_3( t, w, f, ... )  DECL_WIDTH_BIT_FIELDS_2( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define  DECL_WIDTH_BIT_FIELDS_4( t, w, f, ... )  DECL_WIDTH_BIT_FIELDS_3( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define  DECL_WIDTH_BIT_FIELDS_5( t, w, f, ... )  DECL_WIDTH_BIT_FIELDS_4( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define  DECL_WIDTH_BIT_FIELDS_6( t, w, f, ... )  DECL_WIDTH_BIT_FIELDS_5( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define  DECL_WIDTH_BIT_FIELDS_7( t, w, f, ... )  DECL_WIDTH_BIT_FIELDS_6( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define  DECL_WIDTH_BIT_FIELDS_8( t, w, f, ... )  DECL_WIDTH_BIT_FIELDS_7( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define  DECL_WIDTH_BIT_FIELDS_9( t, w, f, ... )  DECL_WIDTH_BIT_FIELDS_8( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define DECL_WIDTH_BIT_FIELDS_10( t, w, f, ... )  DECL_WIDTH_BIT_FIELDS_9( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define DECL_WIDTH_BIT_FIELDS_11( t, w, f, ... ) DECL_WIDTH_BIT_FIELDS_10( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define DECL_WIDTH_BIT_FIELDS_12( t, w, f, ... ) DECL_WIDTH_BIT_FIELDS_11( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define DECL_WIDTH_BIT_FIELDS_13( t, w, f, ... ) DECL_WIDTH_BIT_FIELDS_12( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define DECL_WIDTH_BIT_FIELDS_14( t, w, f, ... ) DECL_WIDTH_BIT_FIELDS_13( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define DECL_WIDTH_BIT_FIELDS_15( t, w, f, ... ) DECL_WIDTH_BIT_FIELDS_14( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define DECL_WIDTH_BIT_FIELDS_16( t, w, f, ... ) DECL_WIDTH_BIT_FIELDS_15( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define DECL_WIDTH_BIT_FIELDS_17( t, w, f, ... ) DECL_WIDTH_BIT_FIELDS_16( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define DECL_WIDTH_BIT_FIELDS_18( t, w, f, ... ) DECL_WIDTH_BIT_FIELDS_17( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define DECL_WIDTH_BIT_FIELDS_19( t, w, f, ... ) DECL_WIDTH_BIT_FIELDS_18( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define DECL_WIDTH_BIT_FIELDS_20( t, w, f, ... ) DECL_WIDTH_BIT_FIELDS_19( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define DECL_WIDTH_BIT_FIELDS_21( t, w, f, ... ) DECL_WIDTH_BIT_FIELDS_20( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define DECL_WIDTH_BIT_FIELDS_22( t, w, f, ... ) DECL_WIDTH_BIT_FIELDS_21( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define DECL_WIDTH_BIT_FIELDS_23( t, w, f, ... ) DECL_WIDTH_BIT_FIELDS_22( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define DECL_WIDTH_BIT_FIELDS_24( t, w, f, ... ) DECL_WIDTH_BIT_FIELDS_23( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define DECL_WIDTH_BIT_FIELDS_25( t, w, f, ... ) DECL_WIDTH_BIT_FIELDS_24( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define DECL_WIDTH_BIT_FIELDS_26( t, w, f, ... ) DECL_WIDTH_BIT_FIELDS_25( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define DECL_WIDTH_BIT_FIELDS_27( t, w, f, ... ) DECL_WIDTH_BIT_FIELDS_26( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define DECL_WIDTH_BIT_FIELDS_28( t, w, f, ... ) DECL_WIDTH_BIT_FIELDS_27( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define DECL_WIDTH_BIT_FIELDS_29( t, w, f, ... ) DECL_WIDTH_BIT_FIELDS_28( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define DECL_WIDTH_BIT_FIELDS_30( t, w, f, ... ) DECL_WIDTH_BIT_FIELDS_29( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define DECL_WIDTH_BIT_FIELDS_31( t, w, f, ... ) DECL_WIDTH_BIT_FIELDS_30( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )
#define DECL_WIDTH_BIT_FIELDS_32( t, w, f, ... ) DECL_WIDTH_BIT_FIELDS_31( t, w, __VA_ARGS__ );  DECL_BIT_FIELDS_1( t, f, w )

// the following macros define sets of bit fields with a common type
// the t parameter is the type to use for all bit fields
// the fx parameter is the name of the bit fields
// the wx parameter is the width of bit field fx
// Example: DECL_BIT_FIELDS_3( unsigned char, a, 2, , 5, b, 1 );
// effectively yeilds...
//   unsigned char a : 2;
//   unsigned char   : 5;
//   unsigned char b : 1;
// NOTE: the second field has no name and is therefore a reserved field
#define DECL_BIT_FIELDS_1(  t, f, w ) t f : w
#define DECL_BIT_FIELDS_2(  t, f, w, ... )  DECL_BIT_FIELDS_1( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_3(  t, f, w, ... )  DECL_BIT_FIELDS_2( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_4(  t, f, w, ... )  DECL_BIT_FIELDS_3( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_5(  t, f, w, ... )  DECL_BIT_FIELDS_4( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_6(  t, f, w, ... )  DECL_BIT_FIELDS_5( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_7(  t, f, w, ... )  DECL_BIT_FIELDS_6( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_8(  t, f, w, ... )  DECL_BIT_FIELDS_7( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_9(  t, f, w, ... )  DECL_BIT_FIELDS_8( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_10( t, f, w, ... )  DECL_BIT_FIELDS_9( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_11( t, f, w, ... ) DECL_BIT_FIELDS_10( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_12( t, f, w, ... ) DECL_BIT_FIELDS_11( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_13( t, f, w, ... ) DECL_BIT_FIELDS_12( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_14( t, f, w, ... ) DECL_BIT_FIELDS_13( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_15( t, f, w, ... ) DECL_BIT_FIELDS_14( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_16( t, f, w, ... ) DECL_BIT_FIELDS_15( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_17( t, f, w, ... ) DECL_BIT_FIELDS_16( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_18( t, f, w, ... ) DECL_BIT_FIELDS_17( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_19( t, f, w, ... ) DECL_BIT_FIELDS_18( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_20( t, f, w, ... ) DECL_BIT_FIELDS_19( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_21( t, f, w, ... ) DECL_BIT_FIELDS_20( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_22( t, f, w, ... ) DECL_BIT_FIELDS_21( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_23( t, f, w, ... ) DECL_BIT_FIELDS_22( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_24( t, f, w, ... ) DECL_BIT_FIELDS_23( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_25( t, f, w, ... ) DECL_BIT_FIELDS_24( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_26( t, f, w, ... ) DECL_BIT_FIELDS_25( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_27( t, f, w, ... ) DECL_BIT_FIELDS_26( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_28( t, f, w, ... ) DECL_BIT_FIELDS_27( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_29( t, f, w, ... ) DECL_BIT_FIELDS_28( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_30( t, f, w, ... ) DECL_BIT_FIELDS_29( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_31( t, f, w, ... ) DECL_BIT_FIELDS_30( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#define DECL_BIT_FIELDS_32( t, f, w, ... ) DECL_BIT_FIELDS_31( t, __VA_ARGS__ ); DECL_BIT_FIELDS_1( t, f, w )
#endif
