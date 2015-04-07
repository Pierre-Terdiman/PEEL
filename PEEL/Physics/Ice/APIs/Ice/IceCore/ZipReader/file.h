#ifndef _FILE_INCLUDE
#define _FILE_INCLUDE

#define	MAX_NAME_SIZE	256

	#pragma pack( push, before_ZIPStructs )
	#pragma pack(1)

	#define ZIPHeaderSig	0x04034b50
	typedef struct  {
		LONG	Signature;		   //  (0x04034b50)
		WORD	Version;
		WORD	Flags;
		WORD	Method;
		LONG	LastMod;
		LONG	CRC32;
		LONG	CompressedSize;
		LONG	UnCompressedSize;
		WORD	FileNameLength;
		WORD	ExtraLength;
	} ZIPHeader;

	#define ZIPCtrlHeaderSig	0x02014b50
	typedef struct  {
		LONG	Signature;		   //  (0x02014b50)
		WORD	VersionMade;
		WORD	VersionNeeded;
		WORD	Flags;
		WORD	Method;
		LONG	LastMod;
		LONG	CRC32;
		LONG	CompressedSize;
		LONG	UnCompressedSize;
		WORD	FileNameLength;
		WORD	ExtraLength;
		WORD	CommentLength;
		WORD	StartDisk;
		WORD	IniternalAttribs;
		LONG	ExternalAttribs;
		LONG	Offset;
	} ZIPCtrlHeader;

	#define ZIPEndSig			0x06054b50
	typedef struct  {
		LONG	Signature;		   //  (0x06054b50)
		WORD	DiskNumber;
		WORD	StartDiskNumber;
		WORD	FilesOnDisk;
		WORD	Files;
		LONG	Size;
		LONG	Offset;
		WORD	CommentLength;
	} ZIPEnd;

	#pragma pack( pop, before_ZIPStructs )

	typedef struct PackedFile_s{
		char			name[MAX_NAME_SIZE];			// FIXME: static allocation
		int				offset;
		int				size;
	} PackedFile;

	typedef struct Pack_s {
		char*		name;
		int			nFiles;
		PackedFile* files;
		Pack_s*		next;
		Constants*	mConstants;
	}Pack;

	char* UnZip(FILE* h);

	Pack* GetArchives();

	class FileEnvironment
	{
	public:
		static bool Shut(void);
		static bool LoadPack(const char* filename);
	};

#endif // _FILE_INCLUDE