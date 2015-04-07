// ******************************************************************** //
//                                                                      //
//  GLOBALS.CPP                                                         //
//  Copyright (c) 1994, Bob Flanders and Michael Holmes                 //
//                                                                      //
//  This module defines global work areas and constant strings for      //
//  PCDEZIP.                                                            //
//                                                                      //
// ******************************************************************** //


/* ******************************************************************** *
 *
 *  Global Variables
 *
 * ******************************************************************** */

ULONG   crc,                                // running crc 32 value
        bhold;                              // bits hold area

long    rsize,                              // remaining size to process
        last_kpal,                          // keep alive message time
        total_size,                         // uncompressed sum
        total_csize;                        // compressed sum

int     rc = 1,                             // errorlevel return code
        pos_count,                          // positional parms count
        sf_count,                           // select file parms count
        e_count,                            // end of data counter/flag
        total_files;                        // files processed

UINT    sb_size= 0;                         // sliding buffer size

BYTE    bsize,                              // bits left to process
       *sb=NULL, *sbp=NULL, *sbe=NULL;      // sliding buffer, ptr, and end

char   *init_path,                          // start up drive and path
      **pos_parms,                          // positional parms array
        spin,                               // keep alive character index
        zfn[MAX_PATH],                      // zip filename and path
        output_path[MAX_PATH],              // base output path
        sw_dirs,                            // create directory switch
        sw_freshen,                         // freshen target files switch
        sw_new,                             // retrieve newer files switch
        sw_overwrite,                       // overwrite files switch
        sw_test,                            // test file integrity switch
        sw_view,                            // view ZIP directory option
        sw_extract,                         // extract file option
       *sw_exclude;                         // exclude files option

FILE   *ifile,                              // input file
       *ofile;                              // output file


/* ******************************************************************** *
 *
 *  Messages and Strings
 *
 * ******************************************************************** */

char    copyright[]    = "PCDEZIP v1.00 þþ Copyright (c) 1994, "
                         "Bob Flanders and Michael Holmes\n"
                         "First Published in PC Magazine "
                         "January 11, 1994\n\n",
        help[]         = "usage:  PCDEZIP  [switches]  zipfile  "
                         "[target\\]  [filespec [...]]\n\n"
                         "switches: -f  freshen existing files\n"
                         "\t  -n  extract new and updated files\n"
                         "\t  -o  do not prompt on overwrite\n"
                         "\t  -v  view ZIP directory\n"
                         "\t  -d  create directories\n"
                         "\t  -t  test file integrity\n\n"
                         "zipfile   is source .ZIP file\n\n"
                         "target\\   is the target directory;"
                         " trailing backslash required\n\n"
                         "filenames are files to be extracted;"
                         " wildcards supported\n\n",
        bad_op[]       = "Invalid parameter -%s\n",
        bad_value[]    = "Missing value for -%c parameter\n",
        bad_switch[]   = "Invalid syntax at %s\n",
        no_memory[]    = "No memory available\n",
        no_zip_found[] = "ZIP file: %s was not found\n",
        done[]         = "PCDEZIP completed normally\n",
        stop_here[]    = "\nStopping at user's request\n",
       *compress[10] = { "unknown", "stored", "shrunk", "reduce1",
                         "reduce2", "reduce3", "reduce4",
                         "imploded", "tokenized", "deflated" },
       *decompr[10]  = { "unknown", "copy", "unshrink", "expand",
                         "expand", "expand", "expand",
                         "explod", "", "inflat" },
        view_hdr[]     = "Processing: %s\n\n"
                         "  Length   Method   Size  Ratio   Date   Time   Name\n"
                         "  ------  -------- ------ ----- -------- -----  ----\n",
        view_line[]    = "%8ld  %8.8s%7ld %3d%%  %s %c%s%s\n",
        view_nextline[]= "\n\t\t",
        view_trailer[] = "  ------           ------ -----                 ----\n"
                         "%8ld%9.9s%8ld %3d%%%16.16s%5d\n\n%s",
        view_encrypt[] = "ø Marks encrypted files\n",
        view_none[]    = "\nNo files were selected\n",
        not_supported[]= "File type not supported\n",
        extract_msg[]  = "Extracting from",
        extract_test[] = "Testing",
        extract_hdr[]  = "%s: %s\n\n",
        extract_line[] = "%11.11sing: %s   ",
        extract_skip[] = "Bypassing unknown compress type\n",
        extract_encr[] = "Encrypted File: %s   * Bypassed *\n",
        extract_ec[]   = "Extraction",
        extract_tc[]   = "Testing",
        extract_done[] = " \n%s Complete: %d files processed\n",
        extract_none[] = "\nNothing extracted\n",
        old_file[]     = " %s\n   Existing:%8ld %s        ",
        new_file[]     = "Zip:%8ld %s\n",
        overwrite_q[]  = "   Overwrite existing file? (Y/n/all/stop) ",
        kpal_msg[]     = "%c\b",
        spin_char[]    = "/-\\|",
        dir_error[]    = "Output directory not available\n",
        invalid_dir[]  = "Unable to create output directory\n",
        open_error[]   = "Error opening output file\n",
        read_error[]   = "Error reading ZIP file\n",
        write_error[]  = "Error writing output file\n",
        data_error[]   = "Data integrity error -- run stopped\n",
        overflow_msg[] = "Decode stack overflow while unshrinking\n",
        crc_error[]    = "CRC error -- run stopped\n",
        test_ok[]      = "Tested ok\n";



/* ******************************************************************** *
 *
 *  Command Line Switches
 *
 * ******************************************************************** */

struct  cmd_parm                            // command line parm entry
    {
    char cp_ltr,                                // switch letter
        *cp_entry,                              // pointer to data
         cp_flag;                               // flag
                                                //  0 = switch flag    -x
                                                //  1 = switch w/data  -x999
    };

struct  cmd_parm parm_table[] =             // command line parms
    {
    { 'D', &sw_dirs,             0 },           // D - create directories
    { 'F', &sw_freshen,          0 },           // F - freshen target files
    { 'N', &sw_new,              0 },           // N - only newer files
    { 'O', &sw_overwrite,        0 },           // O - overwrite files
    { 'T', &sw_test,             0 },           // T - test file integrity
    { 'V', &sw_view,             0 },           // V - view ZIP directory
    { 'E', &sw_extract,          0 }            // E - extract file(s)
 // { 'X', (char *) &sw_exclude, 1 }            // X - exclude files
    };



/* ******************************************************************** *
 *
 *  Split Filename Structure
 *
 * ******************************************************************** */

struct  split_filename                      // split filename entry
    {
    char path[MAX_PATH],                        // full filename
         drv[3],                                // drive letter
         dir[MAX_PATH],                         // directory name
         fname[10],                             // filename
         ext[5];                                // ..and extension
    };

typedef struct split_filename SFS;          // structure shorthand

#define SPLIT(x)    fnsplit(x.path,         /* split file into its */   \
                        x.drv,  x.dir,      /* basic components    */   \
                        x.fname, x.ext);

SFS *sf,                                    // select files
     wf,                                    // working file
     ef;                                    // exclude file



/* ******************************************************************** *
 *
 *  Inflate Constants
 *
 *  These tables are taken from PKZIP's appnote.txt.
 *
 * ******************************************************************** */


UINT bll[] =                                // bit length code lengths
    {
    16, 17, 18, 0, 8, 7, 9, 6, 10, 5,
    11, 4, 12, 3, 13, 2, 14, 1, 15
    };

UINT cll[] =                                // copy lengths for literal ..
    {                                       // ..codes 257 thru 285
    3, 4, 5, 6, 7, 8, 9, 10, 11, 13,
    15, 17, 19, 23, 27, 31, 35, 43,
    51, 59, 67, 83, 99, 115, 131, 163,
    195, 227, 258, 0, 0
    };

UINT cle[] =                                // extra bits for literal
    {                                       // ..codes 257 thru 285
    0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
    1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4,
    4, 4, 5, 5, 5, 5, 0, 99, 99
    };

UINT cdo[] =                                // copy distance offsets
    {                                       // ..for codes 0 thru 29
    1, 2, 3, 4, 5, 7, 9, 13, 17, 25,
    33, 49, 65, 97, 129, 193, 257, 385,
    513, 769, 1025, 1537, 2049, 3073,
    4097, 6145, 8193, 12289, 16385, 24577
    };

UINT cde[] =                                // copy extra bits for distance
    {                                       // ..codes 0 thru 29
    0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4,
    5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10,
    11, 11, 12, 12, 13, 13
    };



/* ******************************************************************** *
 *
 *  malloc_chk() -- allocate memory with error processing
 *
 * ******************************************************************** */

void   *malloc_chk(int n)                  // size of block
{
	void   *s;                                  // temporary pointers
	if (NOT (s = malloc(n)))                    // q. enough memory?
		//Error("[Unzip] Reservando memoria\n");	// a. no .. give error msg

	memset((char *) s,0,(long) n);		   // else .. clear to nulls
	return(s);                                // ..and return w/address

}

/* ******************************************************************** *
 *
 *  quit_with( char * )
 *
 * ******************************************************************** */
void	quit_with( char *error ) 
{
	//Error( error );
}