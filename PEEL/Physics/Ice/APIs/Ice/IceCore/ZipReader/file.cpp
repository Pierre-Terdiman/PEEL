///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Precompiled Header
#include "StdAfx.h"

using namespace IceCore;

#include "file.h"

static Pack* gArchives = null;

Pack* GetArchives()
{
	return gArchives;
}

bool FileEnvironment::LoadPack(const char* filename)
{
	if(!filename)	return false;

	FILE* fp = fopen(filename, "rb");
	if(!fp)
		return false;

	ZIPEnd End;
	// Read the final header
	fseek(fp, -22, SEEK_END);
	fread(&End,  sizeof(End), 1,  fp);
	if(End.Signature != ZIPEndSig)
	{
		fclose(fp);
		return false;
	}

	ZIPCtrlHeader	File;

	Pack* packet = new Pack;
	packet->next = gArchives;
	packet->mConstants = ICE_NEW(Constants);
	gArchives = packet;
	packet->name = new char[strlen(filename)+1];
	strcpy(packet->name, filename);
	packet->nFiles = End.FilesOnDisk;
	packet->files = new PackedFile[End.FilesOnDisk];
	
	// Read the header of each file
	fseek(fp, End.Offset, SEEK_SET);
	int	nPackedFiles = 0;	// Number of files in the archive
	for(int n=0; n < End.FilesOnDisk; n++)
	{
		fread(&File, sizeof(File), 1, fp);
		if(File.Signature != ZIPCtrlHeaderSig)
		{
			fclose(fp);
			return false;
		}

		packet->files[n].size=File.UnCompressedSize;
		packet->files[n].offset=File.Offset;
		ZeroMemory(packet->files[n].name, MAX_NAME_SIZE);	// ###
		ASSERT(File.FileNameLength<MAX_NAME_SIZE);
		fread(packet->files[n].name, 1, File.FileNameLength, fp);
		nPackedFiles++;

		// Add to constant
		packet->mConstants->AddConstant(packet->files[n].name, n);

		fseek(fp, File.ExtraLength + File.CommentLength, SEEK_CUR);
	}

	fclose(fp);

	return true;
}


bool FileEnvironment::Shut(void)
{
	Pack *p, *pnext=NULL;
	for(p=gArchives; p; p=pnext)
	{
		delete [] p->name;
		delete [] p->files;
		DELETESINGLE(p->mConstants);
		pnext=p->next;
		delete p;
	}
	gArchives = null;
	return true;
}
