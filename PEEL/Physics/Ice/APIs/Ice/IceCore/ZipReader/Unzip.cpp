///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Precompiled Header
#include "StdAfx.h"

using namespace IceCore;

//////////////////////////////////////////////////////////////////////
// Unzip.c: Funciones para la descompresión de un paquete
//			de un fichero ZIP.
//		Code extremely based on PCDEZIP.
//////
//
//  PCDEZIP.CPP þþ Copyright 1994, Bob Flanders and Michael Holmes      
//  First Published in PC Magazine January 11, 1994                     
//                                                                      
//  PCDEZIP decompresses .ZIP files created with PKZIP through
//  version 2.04g.  PCDEZIP provides syntax and switch compatibilty
//  with both Michael Mefford's PCUNZP and Phil Katz's PKUNZIP.         
//
//////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <windows.h>

#pragma warning( disable : 4005 )
#pragma warning( disable : 4018 )
#pragma warning( disable : 4305 )
#pragma warning( disable : 4309 )
#undef	FREE
#undef	MAX_PATH

#pragma	 pack(1)
#include "unzip.h"
#include "crc32.h"
#include "globals.h"

/*--------------------------------------------------------*
**	Declaraciones de datos globales.
**-------------------------------------------------------*/
char	*ZIPBuffer = NULL;
char	*ZIPOut	= NULL;


/*--------------------------------------------------------*
**	Declaraciones de funciones
**-------------------------------------------------------*/
int     extract_open(LF *lfp);
void    extract_flush(void);
UINT    lookat_code(int bits);
UINT    get_code(int bits);
UINT    get_byte(void);
void    store_char(char c);
void    extract_copy(ULONG len);
int     exp_cmp(const void *a,             
                const void *b);
UINT    exp_read(SFT *sf);
void    exp_load_tree(SFT *sf,        // base S-F tree pointer
                      int n);         // number of entries
int     inf_build_tree(UINT *b,       // code lengths in bits
                       UINT n,        // number of codes
                       UINT s,        // number of simple codes
                       UINT *d,       // base values for non-simple
                       UINT *e,       // list of extra bits
                       HUFF **t,      // resulting table
                       UINT *m);      // maximum lookup bits
void    inf_free(HUFF *t);
int     inf_fixed(void);
int     inf_dynamic(void);
void    extract_inflate(LF *lfp);     // local file pointer
void    extract_explode(LF *lfp);     // local file pointer
void    extract_expand(LF *lfp);      // local file pointer
void    extract_shrunk(void);         // local file pointer
void    extract_stored(void);         // local file pointer
void    extract_zip(int fnc,          // function
                    LF *lfp);         // local file header pointer


/**********************************************************************
*	char *UnZip( FILE *h )
*
***********************************************************************/
char *UnZip( FILE *h )
{
	LF	lfp;
	ZIPBuffer = ZIPOut = NULL;
	ifile = h;	
	fread( &lfp, sizeof(lfp) , 1 , ifile );
	fseek( ifile , lfp.lf_fn_len , SEEK_CUR );
	fseek( ifile , lfp.lf_ef_len , SEEK_CUR );

	extract_zip(0,&lfp);
	extract_zip(1,&lfp);
	extract_zip(2,&lfp);

	return ZIPBuffer;
}

/* ******************************************************************** *
 *
 *  extract_open() -- open the destination file for output
 *
 *  Returns: 0 = output file opened
 *           1 = skip file expansion
 *
 * ******************************************************************** */

int     extract_open(LF *lfp)               // local file pointer
{
	int		size = lfp->lf_size;
	ZIPBuffer = ZIPOut = (char *)malloc_chk(  size+1 );
	return(0);             // ..and process this file
}


/* ******************************************************************** *
 *
 *  store_char() -- store character in output file
 *
 * ******************************************************************** */

void    store_char(char c)    // character to store
{
	crc_32(c, &crc);          // determine current crc 32
	*sbp = c;                                   // a. yes .. put char in buffer

	if (++sbp >= sbe)                           // q. past end of buffer?
	{											// a. yes .. write buffer
		memcpy( ZIPOut, sb, sb_size );
		ZIPOut += sb_size;			            
		sbp = sb;                               // ..then back to start again
	}
}

/* ******************************************************************** *
 *
 *  extract_flush() -- write output buffer as needed
 *
 * ******************************************************************** */

void    extract_flush(void)
{
	if (sbp != sb)                              // q. local buffer need flushing?
    {											// a. yes .. flush local buffer
		memcpy( ZIPOut, sb, (UINT) (sbp - sb) );
		ZIPOut += (UINT) (sbp - sb);
    }
}



/* ******************************************************************** *
 *
 *  lookat_code() -- look at the next code from input file
 *
 * ******************************************************************** */

UINT    lookat_code(int bits)               // nbr of bits
{
static
UINT    bit_mask[] =                        // bit masks
    {
    0x0000,
    0x0001, 0x0003, 0x0007, 0x000f,
    0x001f, 0x003f, 0x007f, 0x00ff,
    0x01ff, 0x03ff, 0x07ff, 0x0fff,
    0x1fff, 0x3fff, 0x7fff, 0xffff
    };


while (bsize < bits)                        // get enough bits
    {
    if (rsize > 0)                              // q. anything left to process?
        {
        bhold |= (ULONG)fgetc(ifile) << bsize;  // a. yes .. get a character
        bsize += 8;                             // ..and increment counter
        rsize--;                                // ..finally, decrement cnt
        }
     else if (NOT e_count)                  // q. done messing around?
        {
        extract_flush();                    // a. yes .. flush output buffer
        quit_with(data_error);              // ..then quit with an error msg
        }
     else if (e_count > 0)                  // q. down counting?
        {
        e_count--;                          // a. yes .. decriment counter
        return(-1);                         // ..and return all done
        }
     else
        {
        e_count++;                          // else .. count up
        break;                              // ..and exit loop
        }
    }

return(((UINT) bhold) & bit_mask[bits]);    // return just enough bits

}



/* ******************************************************************** *
 *
 *  get_code() -- get next code from input file
 *
 * ******************************************************************** */

UINT    get_code(int bits)                  // nbr of bits in this code
{
UINT    a;                                  // accumulator


a = lookat_code(bits);                      // prepare return value
bhold >>= bits;                             // ..shift out requested bits
bsize -= bits;                              // ..decrement remaining bit count
return(a);                                  // ..and return value

}



/* ******************************************************************** *
 *
 *  get_byte() -- get next byte from input file
 *
 * ******************************************************************** */

UINT    get_byte(void)
{

if (rsize > 0)                              // q. anything left?
    {
    rsize--;                                // a. yes .. decrement count
    return(fgetc(ifile));                   // ..and return w/character
    }
 else
    return(-1);                             // else .. return error
}




/* ******************************************************************** *
 *
 *  extract_copy() -- copy stored data to output stream
 *
 * ******************************************************************** */

void    extract_copy(ULONG len)             // length to copy
{
char   *b,                                  // work buffer pointer
       *p;                                  // work pointer
UINT    csize = 0;                          // current read size


b = (char *) malloc_chk((UINT) COPY_BUFFER);// get an file buffer

while (len)                                 // loop copying file to output
    {
    csize = (len <= COPY_BUFFER) ?          // determine next read size
            (int) len : (UINT) COPY_BUFFER;

    if (fread(b, 1, csize, ifile) != csize) // q. read ok?
        quit_with(read_error);              // a. no .. quit w/error msg

    rsize -= csize;                         // decrement remaining length
    len -= csize;                           // ..and loop control length

    for (p = b; csize--;)                   // for the whole block
        store_char(*p++);                   // ..write output data
    }

free(b);                                    // ..and release file buffer

}



/* ******************************************************************** *
 *
 *  exp_cmp() -- compare the S-F tree entries
 *
 * ******************************************************************** */

int     exp_cmp(const void *a,              // entries to compare
                const void *b)
{

return(strncmp((char *) a, (char *) b, 2)); // compare blen and value

}



/* ******************************************************************** *
 *
 *  exp_load_tree() -- load a single S-F tree
 *
 * ******************************************************************** */

void    exp_load_tree(SFT *sf,              // base S-F tree pointer
                      int n)                // number of entries
{
UINT    c,                                  // current character read
        blen,                               // bit length
        ne,                                 // number of entries
        lb,                                 // last bit increment
        ci,                                 // code increment
        i, j,                               // loop control
        br;                                 // bytes to read
SFT    *p;                                  // work pointer


if ((c = get_byte()) == 0xffff)             // q. read length ok?
    return;                                 // a. no .. just return

br = c + 1;                                 // get number of active elements

for (p = sf, i = 0; br--;)                  // loop thru filling table
    {                                       // ..with lengths
    if ((c = get_byte()) == 0xffff)         // q. get code ok?
        return;                             // a. no .. just return

    blen = (c & 0x0f) + 1;                  // save bit length
    ne = (c >> 4) + 1;                      // ..and nbr of values at blen

    for (j = 0; j < ne; j++, i++, p++)      // loop filling this bit length
        {
        p->blen = blen;                     // save bit length
        p->value = i;                       // ..and the relative entry nbr
        }
    }

qsort((void *) sf, n, sizeof(SFT), exp_cmp);// sort the bit lengths

c = lb = ci = 0;                            // init code, last bit len ..
                                            // ..and code increment
for (p = &sf[n - 1]; p >= sf; p--)          // loop thru tree making codes
    {
    c += ci;                                // build current code
    p->code = c;                            // ..and store in tree

    if (p->blen != lb)                      // q. bit lengths change?
        {
        lb = p->blen;                       // a. yes .. save current value
        ci = (1 << (16 - lb));              // ..and bld new code increment
        }
    }

for (p = sf, j = n; j--; p++)               // loop thru reversing bits
    {
    c = p->code;                            // get code entry

    for (i = 16, ci = 0; i--;)              // loop doing the bit work
        {
        ci = (ci << 1) | (c & 1);           // building the reversed code
        c >>= 1;                            // ..one bit at a time
        }

    p->code = ci;                           // put new code back
    }
}



/* ******************************************************************** *
 *
 *  exp_read() -- read the input stream returning an S-F tree value
 *
 * ******************************************************************** */

UINT    exp_read(SFT *sf)                   // base S-F tree pointer
{
UINT    c,                                  // read code
        code,                               // working bit string
        bits;                               // current nbr of bits
SFT    *p = sf;                             // S-F pointer


for (code = bits = 0;;)                     // loop to find the code
    {
    if ((c = get_code(1)) == 0xffff)        // q. get a bit ok?
        return(-1);                         // a. no .. return w/err code

    c <<= bits++;                           // shift left a little
    code |= c;                              // save next bit

    for (;;)                                // loop to find code
        {
        if (p->blen > bits)                 // q. read enough bits yet?
            break;                          // a. no .. get another one

         else if (p->blen == bits)          // q. read just enough?
            {                               // a. yes .. check entries
            if (p->code == code)            // q. find proper entry?
                return(p->value);           // a. yes .. return w/code
            }

        p++;                                // bump index
        }
    }

#if defined(_MSC_VER)
return(-1);                                 // make MSC C/C++ happy
#endif

}



/* ******************************************************************** *
 *
 *  inf_build_tree() -- build a Huffman tree
 *
 *  Returns: TRUE if error building Huffman tree
 *           FALSE if tree built
 *
 * ******************************************************************** */

#define MAX_BITS    16                      // maximum bits in code
#define CODE_MAX    288                     // maximum nbr of codes in set

int     inf_build_tree(UINT *b,             // code lengths in bits
                       UINT n,              // number of codes
                       UINT s,              // number of simple codes
                       UINT *d,             // base values for non-simple
                       UINT *e,             // list of extra bits
                       HUFF **t,            // resulting table
                       UINT *m)             // maximum lookup bits
{
UINT    a,                                  // code lengths of k
        c[MAX_BITS + 1],                    // bit length count table
        f,                                  // i repeats every f entries
        i, j,                               // loop control
       *p,                                  // work pointer
        v[CODE_MAX],                        // values in order of bit length
        x[MAX_BITS + 1],                    // bit offsets
       *xp,                                 // pointer
        z;                                  // entries in current table
int     g,                                  // max code length
        h,                                  // table level
        k,                                  // loop control
        l,                                  // max bits length
        w,                                  // bits before this table
        y;                                  // number of dummy entries
HUFF    r,                                  // work entry
       *q,                                  // current table entry
       *u[MAX_BITS];                        // table stack


memset(c, 0, sizeof(c));                    // clear table to nulls


for (p = b, i = n; i--;)                    // loop thru table to generate
    c[*p++]++;                              // ..counts for each bit length

if (c[0] == n)                              // q. all zero lengths?
    {
    *t = 0;                                 // a. yes .. clear result..
    *m = 0;                                 // ..pointer and count
    return(FALSE);                          // ..and return all ok
    }

                                            // find min and max code lengths
for (l = *m, j = 1; j <= MAX_BITS; j++)     // loop to find minimum code len
    if (c[j])                               // q. find the min code length?
        break;                              // a. yes .. exit the loop

k = j;                                      // save minimum code length

if ((UINT) l < j)                           // q. minimum greater than lookup?
    l = j;                                  // a. yes .. set up new lookup

for (i = MAX_BITS; i; i--)                  // loop to find max code length
    if (c[i])                               // q. find a used entry?
        break;                              // a. yes .. exit loop

g = i;                                      // save maximum code length

if ((UINT) l > i)                           // q. lookup len greater than max?
    l = i;                                  // a. yes .. set up new look len

*m = l;                                     // return new lookup to caller


for (y = 1 << j; j < i; j++, y <<= 1)       // loop to adjust last length codes
    if ((y -= c[j]) < 0)                    // q. more codes than bits?
        return(TRUE);                       // a. yes .. return w/error flag

if ((y -= c[i]) < 0)                        // q. more than max entry's count?
    return(TRUE);                           // a. yes .. return w/error flag

c[i] += y;                                  // adjust last length code


x[1] = j = 0;                               // initialize code stack
for (p = c + 1, xp = &x[2], i = g; i--;)    // loop thru generating offsets
    *xp++ = (j += *p++);                    // ..into the entry for each length


for (p = b, i = 0; i < n; i++)              // make table of value in order ..
    if ((j = *p++) != 0)                    // ..by bit lengths
        v[x[j]++] = i;


x[0] = i = 0;                               // first Huffman code is all zero
p = v;                                      // use the values array
h = -1;                                     // no tables yet, level = -1
w = -l;                                     // bits decoded
q = u[0] = (HUFF *) 0;                      // clear stack and pointer
z = 0;                                      // ..and number of entries

for (; k <= g; k++)                         // loop from min to max bit lengths
    {
    for (a = c[k]; a--;)                    // process entries at this bit len
        {
        while (k > w + l)                   // build up tables to k length
            {
            h++;                            // increment table level
            w += l;                         // add current nbr of bits

            z = (z = g - w) > (UINT) l      // determine the number of entries
                    ? l : z;                // ..in the current table

            if ((f = 1 << (j = k - w))      // q. k-w bit table contain enough?
                    > a + 1)
                {
                f -= a + 1;                 // a. too few codes for k-w bit tbl
                xp = c + k;                 // ..deduct codes from patterns left

                while (++j < z)             // loop to build upto z bits
                    {
                    if ((f <<= 1) <= *++xp) // q. large enough?
                        break;              // a. yes .. to use j bits

                    f -= *xp;               // else .. remove codes
                    }
                }

            z = 1 << j;                     // entries for j-bit table


            q = (HUFF *) malloc_chk(        // get memory for new table
                    (z + 1) * sizeof(HUFF));

            *t = q + 1;                     // link to main list
            *(t = &(q->v.table)) = 0;       // clear ptr, save address
            u[h] = ++q;                     // table starts after link


            if (h)                          // q. is there a last table?
                {
                x[h] = i;                   // a. yes .. save pattern
                r.blen = (BYTE) l;          // bits to dump before this table
                r.eb = (BYTE) (16 + j);     // bits in this table
                r.v.table = q;              // pointer to this table
                j = i >> (w - l);           // index to last table
                u[h-1][j] = r;              // connect to last table
                }
            }


        r.blen = (BYTE) (k - w);            // set up table entry

        if (p >= v + n)                     // q. out of values?
            r.eb = 99;                      // a. yes .. set up invalid code
         else if (*p < s)                   // q. need extra bits?
            {
            r.eb = (BYTE) (*p < 256         // a. yes .. set up extra bits
                    ? 16 : 15);             // 256 is end-of-block code
            r.v.code = *p++;                // simple code is just the value
            }
         else
            {
            r.eb = (BYTE) e[*p - s];        // non-simple--look up in lists
            r.v.code = d[*p++ - s];
            }


        f = 1 << (k - w);                   // fill code-like entries with r

        for (j = i >> w; j < z; j += f)
            q[j] = r;


        for (j = 1 << (k - 1); i & j; j >>= 1)  // backwards increment the
            i ^= j;                             // ..k-bit code i

        i ^= j;


        while ((i & ((1 << w) - 1)) != x[h])
            {
            h--;                            // decrement table level
            w -= l;                         // adjust bits before this table
            }
        }
    }

return(FALSE);                              // return FALSE, everything ok

}



/* ******************************************************************** *
 *
 *  inf_free() -- free malloc'd Huffman tables
 *
 * ******************************************************************** */

void    inf_free(HUFF *t)                   // base table to free
{
HUFF   *p;                                  // work pointer


while (t)                                   // loop thru freeing memory
    {
    p = (--t)->v.table;                     // get next table address
    free(t);                                // free current table
    t = p;                                  // establish new base pointer
    }
}



/* ******************************************************************** *
 *
 *  inf_codes() -- inflate the codes using the Huffman trees
 *
 * ******************************************************************** */

int     inf_codes(HUFF *tl,                 // literal table
                  HUFF *td,                 // distance table
                  int  bl,                  // literal bit length
                  int  bd)                  // distance bit length
{
UINT    c,                                  // current retrieved code
        e,                                  // extra bits
        n, d;                               // length and distance
BYTE   *p;                                  // work pointer
HUFF   *t;                                  // current Huffman tree

for (;;)                                    // loop till end of block
    {
    c = lookat_code(bl);                    // get some bits

    if ((e = (t = tl + c)->eb) > 16)        // q. in this table?
        do  {                               // a. no .. loop reading codes
            if (e == 99)                    // q. invalid entry?
                return(TRUE);               // a. yes .. return an error

            get_code(t->blen);              // read some bits
            e -= 16;                        // nbr of bits to get
            c = lookat_code(e);             // get some bits
            }
            while ((e = (t = t->v.table + c)->eb) > 16);

    get_code(t->blen);                      // read some processed bits

    if (e == 16)                            // q. literal code?
        store_char(t->v.code);              // a. yes .. output code

     else
        {
        if (e == 15)                        // q. end of block?
            return(FALSE);                  // a. yes .. return all ok

        n = get_code(e) + t->v.code;        // get length code
        c = lookat_code(bd);                // get some bits

        if ((e = (t = td + c)->eb) > 16)    // q. in this table?
            do  {                           // a. no .. loop thru
                if (e == 99)                // q. invalid entry?
                    return(TRUE);           // a. yes .. just return

                get_code(t->blen);          // read some bits
                e -= 16;                    // number of bits to get
                c = lookat_code(e);         // get some bits
                }
                while ((e = (t = t->v.table + c)->eb) > 16);

        get_code(t->blen);                  // read some processed bits
        d = t->v.code + get_code(e);        // get distance value

        if ((sbp - sb) >= d)                // q. backward wrap?
            p = sbp - d;                    // a. no .. just back up a bit
         else
            p = sb_size - d + sbp;          // else .. find at end of buffer

        while (n--)                         // copy previously outputed
            {                               // ..strings from sliding buffer
            store_char(*p);                 // put out each character

            if (++p >= sbe)                 // q. hit the end of the buffer?
                p = sb;                     // a. yes .. back to beginning
            }
        }
    }
}



/* ******************************************************************** *
 *
 *  inf_fixed() -- inflate a fixed Huffman code block
 *
 * ******************************************************************** */

int     inf_fixed(void)
{
UINT    i,                                  // loop control
        bl = 7,                             // bit length for literal codes
        bd = 5,                             // ..and distance codes
        l[288];                             // length list
HUFF   *tl, *td;                            // literal and distance trees


for (i = 0; i < 144; i++)                   // set up literal table
    l[i] = 8;

for (; i < 256; i++)                        // ..fixing up ..
    l[i] = 9;

for (; i < 280; i++)                        // ..all the
    l[i] = 7;

for (; i < 288; i++)                        // ..entries
    l[i] = 8;

if (inf_build_tree(l, 288, 257, cll,        // q. build literal table ok?
            cle, &tl, &bl))
    return(TRUE);                           // a. no .. return with error

for (i = 0; i < 30; i++)                    // set up the distance list
    l[i] = 5;                               // ..to 5 bits

if (inf_build_tree(l, 30, 0, cdo, cde,      // q. build distance table ok?
            &td, &bd))
    return(TRUE);                           // a. no .. return with error

if (inf_codes(tl, td, bl, bd))              // q. inflate file ok?
    return(TRUE);                           // a. no .. return with error

inf_free(tl);                               // free literal trees
inf_free(td);                               // ..and distance trees
return(FALSE);                              // ..and return all ok

}



/* ******************************************************************** *
 *
 *  inf_dynamic() -- inflate a dynamic Huffman code block
 *
 * ******************************************************************** */

int     inf_dynamic(void)
{
UINT    c,                                  // code read from input stream
        i, j,                               // loop control
        l,                                  // last length
        n,                                  // nbr of lengths to get
        bl, bd,                             // literal and distance bit len
        nl, nd,                             // literal and distance codes
        nb,                                 // nbr of bit length codes
        ll[286 + 30];                       // literal length and dist codes
HUFF   *tl, *td;                            // literal and distance trees


nl = get_code(5) + 257;                     // get nbr literal len codes
nd = get_code(5) + 1;                       // ..and the nbr dist len codes
nb = get_code(4) + 4;                       // ..and nbr of of bit lengths

for (j = 0; j < nb; j++)                    // read in bit length code
    ll[bll[j]] = get_code(3);               // set up bit lengths

for (; j < 19; j++)                         // loop thru clearing..
    ll[bll[j]] = 0;                         // ..other lengths

bl = 7;                                     // set literal bit length

if (inf_build_tree(ll, 19, 19,              // q. build decoding table for
            0, 0, &tl, &bl))                // ..trees using 7 bit lookup ok?
    return(TRUE);                           // a. no .. return with error


n = nl + nd;                                // number of lengths to get

for (i = l = 0; i < n;)                     // get literal and dist code lengths
    {
    c = lookat_code(bl);                    // get some bits
    j = (td = tl + c)->blen;                // get length code from table
    get_code(j);                            // use those bits
    j = td->v.code;                         // ..then get code from table

    if (j < 16)                             // q. save length?
        ll[i++] = l = j;                    // a. yes .. also save last length
     else if (j == 16)                      // q. repeat last length 3 to 6x?
        {
        j = get_code(2) + 3;                // get repeat length code

        if (i + j > n)                      // q. past end of array?
            return(TRUE);                   // a. yes .. return with error

        while (j--)                         // else .. loop filling table
            ll[i++] = l;                    // ..with last length
        }
     else if (j == 17)                      // q. 3 to 10 zero length codes?
        {
        j = get_code(3) + 3;                // a. yes .. get repeat code

        if (i + j > n)                      // q. past end of array?
            return(TRUE);                   // a. yes .. return with error

        while (j--)                         // else .. loop filling table
            ll[i++] = 0;                    // ..with zero length

        l = 0;                              // ..and save new last length
        }
     else                                   // else .. j == 18 and
        {                                   // ..generate 11 to 138 zero codes
        j = get_code(7) + 11;               // get repeat code

        if (i + j > n)                      // q. past end of array?
            return(TRUE);                   // a. yes .. return with error

        while (j--)                         // else .. loop filling table
            ll[i++] = 0;                    // ..with zero length

        l = 0;                              // ..and save new last length
        }
    }

inf_free(tl);                               // finally, free literal tree

bl = 9;                                     // length of literal bit codes

if (inf_build_tree(ll, nl, 257,             // q. build literal table ok?
            cll, cle, &tl, &bl))
    return(TRUE);                           // a. no .. return with error

bd = 6;                                     // length of distance bit codes

if (inf_build_tree(ll + nl, nd, 0,          // q. build distance table ok?
            cdo, cde, &td, &bd))
    return(TRUE);                           // a. no .. return with error

if (inf_codes(tl, td, bl, bd))              // q. inflate block ok?
    return(TRUE);                           // a. no .. return with error

inf_free(tl);                               // free literal trees
inf_free(td);                               // ..and distance trees
return(FALSE);                              // then finally, return all ok

}



/* ******************************************************************** *
 *
 *  extract_inflate() -- extract a deflated file
 *
 * ******************************************************************** */

void    extract_inflate(LF *lfp)            // local file pointer
{
UINT    c,                                  // current read character
        eoj = 0;                            // end of job flag


if (lfp->lf_flag & LF_FLAG_DDREC)           // q. need data descriptor rec?
    quit_with(not_supported);               // a. yes .. quit w/error msg

e_count = -e_count;                         // set end count to negative

while (NOT eoj)                             // loop till end of job
    {
    eoj = get_code(1);                      // get the eoj bit
    c = get_code(2);                        // ..then get block type

    switch (c)                              // depending on block type
        {
        case 0:                             // 0: stored block
            bsize = 0;                      // flush remaining bits
            c = get_code(16);               // get block length ok

            fgetc(ifile);                   // skip the ..
            fgetc(ifile);                   // ..ones complement word
            rsize -= 2;                     // ..and its count

            extract_copy(c);                // copy bytes to output stream
            break;                          // ..and get next block

        case 1:                             // 1: fixed Huffman codes
            eoj |= inf_fixed();             // process a fixed block
            break;                          // ..then get another block

        case 2:                             // 2: dynamic Huffman codes
            eoj |= inf_dynamic();           // process the dynamic block
            break;                          // ..then get next block

        case 3:                             // 3: unknown type
            quit_with(data_error);          // quit with an error message
        }
    }

}



/* ******************************************************************** *
 *
 *  extract_explode() -- extract an imploded file
 *
 * ******************************************************************** */

void    extract_explode(LF *lfp)            // local file pointer
{
UINT    c,                                  // current read character
        ltf,                                // literal S-F tree available
        db,                                 // dictionary read bits
        d,                                  // distance
        len,                                // ..and length to go back
        mml;                                // minimum match length 3
BYTE   *p;                                  // work dictionary pointer
SFT    *sft, *sft2, *sft3;                  // S-F trees pointers


sft = (SFT *) malloc_chk(                   // get memory for S-F trees
            (256 + 64 + 64) * sizeof(SFT));

sft2 = &sft[256];                           // ..and set up ..
sft3 = &sft[320];                           // ..the base pointers

db = (lfp->lf_flag & LF_FLAG_8K) ? 7 : 6;   // ..and dictionary read in bits

mml = ((ltf = lfp->lf_flag & LF_FLAG_3SF)   // set literal S-F tree available
            != 0) ? 3 : 2;                  // ..and minimum match lengths

if (ltf)                                    // q. literal tree available?
    exp_load_tree(sft, 256);                // a. yes .. load literal tree

exp_load_tree(sft2, 64);                    // ..then load length trees
exp_load_tree(sft3, 64);                    // ..and finally, distance trees

for (;;)                                    // loop processing compressed data
    {
    if ((c = get_code(1)) == 0xffff)        // q. get a bit ok?
        break;                              // a. no .. exit loop

    if (c)                                  // q. encoded literal data?
        {                                   // a. yes .. continue processing
        if (ltf)                            // q. literal S-F tree available?
            {                               // a. yes .. get char from tree
            if ((c = exp_read(sft)) == 0xffff)  // q. get char from tree ok?
                break;                          // a. no .. exit loop
            }
         else if ((c = get_code(8)) == 0xffff)  // q. get next character ok?
            break;                              // a. no .. exit loop

        store_char(c);                      // ..and put char to output stream
        }
     else                                   // else .. use sliding dictionary
        {
        if ((d = get_code(db)) == 0xffff)   // q. get distance code ok?
            break;                          // a. no .. exit loop

        if ((c = exp_read(sft3)) == 0xffff) // q. get distance S-F code ok?
            break;                          // a. no .. exit loop

        d = (d | (c << db)) + 1;            // update distance

        if ((len = exp_read(sft2)) == 0xffff)   // q. get length S-F code ok?
            break;                              // a. no .. exit loop

        if (len == 63)                      // q. get max amount?
            {                               // a. yes .. get another byte
            if ((c = get_code(8)) == 0xffff)// q. get additional len ok?
                break;                      // a. no .. exit loop

            len += c;                       // ..then add to overall length
            }

        len += mml;                         // add in minimum match length

        if ((sbp - sb) >= d)                // q. backward wrap?
            p = sbp - d;                    // a. no .. just back up a bit
         else
            p = sb_size - d + sbp;          // else .. find at end of buffer

        while (len--)                       // copy previously outputed
            {                               // ..strings from sliding buffer
            store_char(*p);                 // put out each character

            if (++p >= sbe)                 // q. hit the end of the buffer?
                p = sb;                     // a. yes .. back to beginning
            }
        }
    }

free(sft);                                  // free S-F trees

}



/* ******************************************************************** *
 *
 *  extract_expand() -- extract a reduced file
 *
 * ******************************************************************** */

void    extract_expand(LF *lfp)             // local file pointer
{
UINT    i, j, k,                            // loop variables
        c,                                  // current character
        save_reduce,                        // reduction character
        reduce_m,                           // mask
        reduce_s,                           // shift value
        exp_len;                            // expand length
BYTE    last_c = 0,                         // last character
        state = 0,                          // state machine indicator
       *p;                                  // work pointer
FS     *fsp,                                // follower sets pointer
       *fse;                                // ..and entry
static
BYTE    reduce[4][2] =                      // reduction mask and lengths
            {
            { 0x7f, 7 },
            { 0x3f, 6 },
            { 0x1f, 5 },
            { 0x0f, 4 }
            },
        len_codes[33] =                     // bit lengths for numbers
            { 1, 1, 1, 2, 2, 3, 3, 3, 3, 4, // this table maps the minimum
              4, 4, 4, 4, 4, 4, 4, 5, 5, 5, // ..number of bits to represent
              5, 5, 5, 5, 5, 5, 5, 5, 5, 5, // ..a value
              5, 5, 5
            };


fsp = (FS *) malloc_chk(256 * sizeof(FS));  // allocate memory for sets

i = lfp->lf_cm - LF_CM_REDUCED1;            // get index into array
reduce_m = reduce[i][0];                    // ..copy over mask
reduce_s = reduce[i][1];                    // ..and shift amount

for (i = 256, fse = &fsp[255]; i--; fse--)  // build follower sets
    {
    if ((j = get_code(6)) == 0xffff)        // q. get a length code ok?
        break;                              // a. no .. exit loop

    fse->set_len = j;                       // save length of set data

    for (p = fse->set; j--; p++)            // set up length in set
        {
        if ((k = get_code(8)) == 0xffff)    // q. get a data code ok?
            break;                          // a. no .. exit loop

        *p = (char) k;                      // save set data
        }
    }

for (;;)                                    // loop till file processed
    {
    fse = &fsp[last_c];                     // current follower set

    if (NOT fse->set_len)                   // q. empty set?
        {                                   // a. yes .. get more input
        if ((c = get_code(8)) == 0xffff)    // q. get a code ok?
            break;                          // a. no .. exit loop
        }
     else
        {
        if ((c = get_code(1)) == 0xffff)    // q. get a code ok?
            break;                          // a. no .. exit loop

        if (c)                              // q. need to get another byte?
            {                               // a. yes .. get another
            if ((c = get_code(8)) == 0xffff)    // q. get a code ok?
                break;                          // a. no .. exit loop
            }
         else
            {
            i = len_codes[fse->set_len];    // get next read bit length

            if ((c = get_code(i)) == 0xffff)// q. get next code ok?
                break;                      // a. no .. exit loop

            c = fse->set[c];                // get encoded character
            }
        }

    last_c = c;                             // set up new last character

    switch (state)                          // based on current state
        {
        case 0:                             // 0: output character
            if (c == EXPLODE_DLE)           // q. DLE character?
                state = 1;                  // a. yes .. change states
             else
                store_char(c);              // else .. output character
            break;                          // ..then process next character


        case 1:                             // 1: store length
            if (NOT c)                      // q. null character?
                {
                store_char(EXPLODE_DLE);    // a. yes .. output a DLE char
                state = 0;                  // ..and change states
                }
             else
                {
                save_reduce = c;            // save character being reduced
                c &= reduce_m;              // clear unused bits
                exp_len = c;                // save length to expand
                state = (c == reduce_m)     // select next state
                        ? 2 : 3;            // ..if char is special flag
                }
            break;                          // ..then process next character

        case 2:                             // 2: store length
            exp_len += c;                   // save length to expand
            state = 3;                      // select next state
            break;                          // ..then get next character

        case 3:                             // 3: expand string
            c = ((save_reduce >> reduce_s)  // compute offset backwards
                    << 8) + c + 1;
            exp_len += 3;                   // set up expansion length

            if ((sbp - sb) >= c)            // q. backward wrap?
                p = sbp - c;                // a. no .. just back up a bit
             else
                p = sb_size - c + sbp;      // else .. find at end of buffer

            while (exp_len--)               // copy previously outputed
                {                           // ..strings from sliding buffer
                store_char(*p);             // put out each character

                if (++p >= sbe)             // q. hit the end of the buffer?
                    p = sb;                 // a. yes .. back to beginning
                }

            state = 0;                      // change state back
            break;                          // ..and process next character
        }
    }

free(fsp);                                  // free follower sets

}



/* ******************************************************************** *
 *
 *  extract_shrunk() -- extract a LZW shrunk file
 *
 * ******************************************************************** */

void    extract_shrunk(void)                // local file pointer
{
int     b_c,                                // base code
        p_c,                                // previous code
        s_c,                                // saved code
        c;                                  // current code
UINT    cnt;                                // decode stack counter
char    cs = 9;                             // code size
SD      *dict,                         // main dictionary
        *d,                            // dictionary entry pointer
        *fd,                           // next free pointer
        *ld;                           // last entry
char    *decode_stack;                 // decode stack


dict = (SD *) malloc_chk(             // allocate dictionary trees
            sizeof(SD) * TABLE_SIZE);
decode_stack = (char *) malloc_chk(   // ..and allocate decode stack
            TABLE_SIZE);

ld = &dict[TABLE_SIZE];                     // get address of last entry

for (fd = d = &dict[257]; d < ld; d++)      // loop thru dictionary
    d->parent_c = FREE;                     // ..and make each one free

store_char(p_c = b_c = get_code(cs));       // get and store a code

for (;;)                                    // inner loop
    {
    cnt = 0;                                // reset decode stack

    if ((s_c = c = get_code(cs)) == -1)     // q. end of data?
        break;                              // a. yes .. exit loop

    if (c == 256)                           // q. special code?
        {                                   // a. yes .. get next code
        if ((c = get_code(cs)) == -1)       // q. get next code ok?
            quit_with(data_error);          // a. no .. quit w/error msg

        if (c == 1)                         // q. 256,1 sequence?
            {
            cs++;                           // a. yes .. increase code size
            continue;                       // ..and get next character
            }
         else if (c == 2)                   // q. clear tree (256,2)?
            {                               // a. yes .. partially clear nodes
            for (d = &dict[257];            // loop thru dictionary..
                        d < fd; d++)        // ..starting past literals
                d->parent_c |= 0x8000;      // ..and mark as unused

            for (d = &dict[257];            // loop again thru dictionary..
                        d < fd; d++)        // ..checking each used node
                {
                c = d->parent_c & 0x7fff;       // get node's next pointer

                if (c >= 257)                   // q. uses another node?
                    dict[c].parent_c &= 0x7fff; // a. yes .. clear target
                }

            for (d = &dict[257];            // loop once more, this time
                        d < fd; d++)        // ..release unneeded entries
                if (d->parent_c & 0x8000)   // q. need to be cleared?
                    d->parent_c = FREE;     // a. yes .. set it to free

            for (d = &dict[257];            // loop thru dictionary to..
                        d < ld; d++)        // ..find the first free node
                if (d->parent_c == FREE)    // q. find a free entry?
                    break;                  // a. yes .. exit loop

            fd = d;                         // save next free dict node
            continue;                       // ..continue with inner loop
            }
        }


    if (c < 256)                            // q. literal code?
        store_char(b_c = c);                // a. yes .. put out literal

     else                                   // else .. must be .gt. 256
        {
        if(dict[c].parent_c == FREE)        // q. using new code?
            {
            decode_stack[cnt++] = b_c;      // a. yes .. store old character
            c = p_c;                        // set up search criteria
            }

        while (c > 255)                     // loop finding entries to use
            {
            d = &dict[c];                   // point to current entry
            decode_stack[cnt++] = d->c;     // put character into stack
            c = d->parent_c;                // get parent's code
            }

        store_char(b_c = c);                // put out first character

        while (cnt)                         // loop outputing from ..
            store_char(decode_stack[--cnt]);// ..decode stack
        }

    fd->parent_c = p_c;                     // store parent's code
    fd->c = b_c;                            // ..and its character
    p_c = s_c;                              // set up new parent code

    while (++fd < ld)                       // loop thru dictionary
        if (fd->parent_c == FREE)           // q. entry free?
            break;                          // a. yes .. done looping
    }

	free(decode_stack);                      // free decode stack
	free(dict);                              // ..and dictionary
}

/* ******************************************************************** *
 *
 *  extract_stored() -- extract a stored file
 *
 * ******************************************************************** */
void    extract_stored(void)                // local file pointer
{

extract_copy(rsize);                        // copy stored data to file

}



/* ******************************************************************** *
 *
 *  extract_zip() -- extract files from a ZIP file
 *
 *  This routine is a sub-function of the scan_zip() routine.  The
 *  extract_zip() routine is call with the following function calls.
 *
 *      0 = initialization call
 *      1 = current file number
 *      2 = completion call
 *
 * ******************************************************************** */

void    extract_zip(int fnc,                // function
                    LF *lfp)                // local file header pointer
{

switch (fnc)                                // based on function number
    {
    case 0:                                 // initialization call
        sb_size = (UINT) BUFFER_SIZE;       // set up the dictionary size
		if(sb==NULL) sb = (BYTE *) malloc_chk(sb_size); // get memory for output buffer
        sbe = &sb[sb_size];                 // set up end of buffer address
        break;                              // ..and return to caller

    case 1:                                 
        if (lfp->lf_flag & LH_FLAG_ENCRYPT) 
        {
			//Error(" No se admiten ficheros encriptados.\n");
			//ASSERT( FALSE );	
            break;                          
        }

        if ( extract_open(lfp) )
            break;                          

        rsize = lfp->lf_csize;              // remaining filesize to process
        bsize = 0;                          // ..bits in byte to process
        e_count = 2;                        // end of data counter/flag
        crc = -1;                           // clear current crc
        sbp = sb;                           // reset next output pointer
        last_kpal = 0;                      // clear keep alive timer
        spin = 0;                           // ..next kpal character index
        memset(sb, 0, sb_size);             // ..and dictionary/output buffer

        switch (lfp->lf_cm)                 // based on compression method
            {
            case LF_CM_STORED:              // stored file
                extract_stored();           // process stored file
                break;                      // ..then do next file

            case LF_CM_SHRUNK:              // LZW shrunk file
                extract_shrunk();           // process shrunk file
                break;                      // ..then do next file

            case LF_CM_REDUCED1:            // reduced file #1
            case LF_CM_REDUCED2:            // reduced file #2
            case LF_CM_REDUCED3:            // reduced file #3
            case LF_CM_REDUCED4:            // reduced file #4
                extract_expand(lfp);        // process reduced file
                break;                      // ..then do next file

            case LF_CM_IMPLODED:            // imploded file
                extract_explode(lfp);       // process imploded file
                break;                      // ..then do next file

            case LF_CM_DEFLATED:            // deflated file
                extract_inflate(lfp);       // process deflated file
                break;                      // ..then do next file

            case LF_CM_TOKENIZED:           // tokenized file
            default:
                return;                     // ..and return to caller
            }

        extract_flush();                    // else .. flush our output buffer        		

        //if (rsize)                          // q. use up input stream?
        //    Error(data_error);				// a. no .. quit w/err message

        crc ^= 0xffffffffL;                 // finalize crc value

//        if (crc != lfp->lf_crc)             // q. crc match?
  //          Error(crc_error);				// a. no .. quit w/err message

/*      if (!sw_test)                        // q. testing files?
		{
			Error("ZIP Test Failed\n");
		}
*/
  
        break;                              // ..and return to caller

    case 2:                                 // completion call
		if(sb!=NULL)
		{
			free(sb);
			sb = NULL;
		}
        break;                              // then return to caller
    }
}

#pragma warning( default : 4005 )
#pragma warning( default : 4018 )
#pragma warning( default : 4305 )
#pragma warning( default : 4309 )  

