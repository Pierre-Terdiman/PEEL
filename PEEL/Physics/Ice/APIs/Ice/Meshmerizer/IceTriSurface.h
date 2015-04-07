///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains a tri-surface class.
 *	\file		IceTriSurface.h
 *	\author		Pierre Terdiman
 *	\date		January, 17, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef ICETRISURFACE_H
#define ICETRISURFACE_H

	// A non-indexed triangle surface
	class MESHMERIZER_API TriSurface
	{
		public:
		// Constructor/Destructor
							TriSurface();
							~TriSurface();

		// Surface init

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Initializes a tri-surface.
		 *	\param		nb_faces	[in] the number of triangles in the surface
		 *	\param		faces		[in] possible list of faces if available
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				bool		Init(udword nb_faces, const Triangle* faces=null);

		// Surface construction

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Adds a triangle to the surface.
		 *	\param		p0		[in] first vertex
		 *	\param		p1		[in] second vertex
		 *	\param		p2		[in] third vertex
		 *	\param		i		[in] face index
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				bool		SetFace(const Point& p0, const Point& p1, const Point& p2, udword i);

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Adds a triangle to the surface.
		 *	\param		triangle	[in] new triangle
		 *	\param		i			[in] face index
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				bool		SetFace(const Triangle& triangle, udword i);

		// Surface destruction

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Frees all used bytes and releases everything.
		 *	\return		Self-reference
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				TriSurface&	Clean();

		// Surface operations

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Cuts a tri-surface by a given plane. This method splits the source surface into two (positive and negative) sub-surfaces.
		 *	\param		cut_plane			[in] plane equation
		 *	\param		positive_surface	[in] the positive destination surface (or null)
		 *	\param		negative_surface	[in] the negative destination surface (or null)
		 *	\param		positive_indices	[in] original indices of positive triangles
		 *	\param		negative_indices	[in] original indices of negative triangles
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				bool		Cut(const Plane& cut_plane, TriSurface* positive_surface, TriSurface* negative_surface, Container* positive_indices=null, Container* negative_indices=null)	const;

				Plane		FindBestDivider();

		// Data access
		inline_	udword		GetNbFaces()		const	{ return mNbFaces;		}
		inline_	Triangle*	GetFaces()			const	{ return mFaces;		}
		inline_	Triangle*	GetFace(udword i)	const	{ return &mFaces[i];	}

		private:
		// Mixed topology-geometry
				udword		mNbFaces;			//!< Number of triangles
				Triangle*	mFaces;				//!< List of non-indexed triangles
	};






	// A triangle class.
	// ### make that a virtual interface to support arbitrary vertices
	class MESHMERIZER_API FatTriangle : public Allocateable
	{
		public:
				Point			mVerts[3];
				Point			mUVs[3];
				Point			mColor[3];
	};

	// A special version with a "fat" vertex....
	class MESHMERIZER_API FatTriSurface
	{
		public:
		// Constructor/Destructor
								FatTriSurface();
								~FatTriSurface();

				bool			Init(udword nb_faces, const FatTriangle* faces=null);
				bool			SetFace(const FatTriangle& triangle, udword i);
				void			Clean();

				bool			Cut(const Plane& cut_plane, FatTriSurface* positive_surface, FatTriSurface* negative_surface, Container* positive_indices=null, Container* negative_indices=null)	const;

		// Data access
		inline_	udword			GetNbFaces()		const	{ return mNbFaces;		}
		inline_	FatTriangle*	GetFaces()			const	{ return mFaces;		}
		inline_	FatTriangle*	GetFace(udword i)	const	{ return &mFaces[i];	}

		private:
		// Mixed topology-geometry
				udword			mNbFaces;			//!< Number of triangles
				FatTriangle*	mFaces;				//!< List of non-indexed triangles
	};

#endif // ICETRISURFACE_H