/**
*	@file
*	@brief	\8Cv\91\AA\83\89\83C\83u\83\89\83\8A\81[							Ver 1.0.9.x
*/
/*============================================================================*/
#ifndef MEASUREMENT_LIBRARY_H_
#define MEASUREMENT_LIBRARY_H_


/*----------------------------------------------------------------------*/
/**
*	@brief	Mixing coefficient\81imlCalMixCoef\8A֐\94\81j\97p\82̓\FC\97͒l
*/
struct tagMlMixingValue
{
	double	radianX;
	double	radianY;

	double	hx45x;
	double	hy45x;
	double	hy45y;
	double	hx45y;

	UINT_8	hxsx;
	UINT_8	hysx;

	INT_32	hx45xL;		//! for Fixed point
	INT_32	hy45xL;		//! for Fixed point
	INT_32	hy45yL;		//! for Fixed point
	INT_32	hx45yL;		//! for Fixed point
};
/**
*	@brief	Mixing coefficient\81imlCalMixCoef\8A֐\94\81j\97p\82̓\FC\97͒l
*/
typedef	struct tagMlMixingValue		mlMixingValue;

/*----------------------------------------------------------------------*/
/**
*	@brief	Lineaity correction\81imlCalLinearCorr\8A֐\94\81j\97p\82̓\FC\97͒l
*/
struct tagMlLinearityValue
{
	INT_32	measurecount;	//! input parameter
	UINT_32	*dacX;			//! input parameter
	UINT_32	*dacY;			//! input parameter

	double	*positionX;
	double	*positionY;
	UINT_16	*thresholdX;
	UINT_16	*thresholdY;

	UINT_32	*coefAXL;		//! for Fixed point
	UINT_32	*coefBXL;		//! for Fixed point
	UINT_32	*coefAYL;		//! for Fixed point
	UINT_32	*coefBYL;		//! for Fixed point
};
/**
*	@brief	Linearity correction\81imlCalLinearCorr\8A֐\94\81j\97p\82̓\FC\97͒l
*/
typedef	struct tagMlLinearityValue		mlLinearityValue;

struct tagMlPoint
{
	double	X;
	double	Y;
};
/**
*	@brief	Linearity correction\81imlCalLinearCorr\8A֐\94\81j\97p\82̓\FC\97͒l
*/
typedef	struct tagMlPoint		mlPoint;


/*----------------------------------------------------------------------*/
/**
*	@brief	\83\89\83C\83u\83\89\83\8A\81[\83G\83\89\81[\83R\81[\83h
*/
enum tagErrorCode
{
	/**! \83G\83\89\81[\96\B3\82\B5\82Ő\B3\8F\ED\8FI\97\B9 */
	ML_OK,

	/**! \83\81\83\82\83\8A\95s\91\AB\93\99\83\81\83\82\83\8A\81[\8A֘A\82̃G\83\89\81[ */
	ML_MEMORY_ERROR,
	/**! \88\F8\90\94\8Ew\92\E8\82̃G\83\89\81[ */
	ML_ARGUMENT_ERROR,
	/**! \88\F8\90\94\82\C9NULL\82\AA\8Ew\97߂\B3\82\EA\82Ă\A2\82\E9\83G\83\89\81[ */
	ML_ARGUMENT_NULL_ERROR,

	/**! \8Ew\92肳\82ꂽ\83f\83B\83\8C\83N\83g\83\8A\82\AA\91\B6\8D݂\B5\82Ȃ\A2\83G\83\89\81[ */
	ML_DIRECTORY_NOT_EXIST_ERROR,
	/**! \89摜\83t\83@\83C\83\8B\82\AA\91\B6\8D݂\B5\82Ȃ\A2\83G\83\89\81[ */
	ML_FILE_NOT_EXIST_ERROR,
	/**! \83t\83@\83C\83\8BIO\83G\83\89\81[ */
	ML_FILE_IO_ERROR,
	/**! \96\A2\8C\9F\8Fo\82̃}\81[\83N\82\AA\97L\82\E8 */
	ML_UNDETECTED_MARK_ERROR,
	/**! \93\AF\82\B6\88ʒu\82\F0\8E\A6\82\B7\83}\81[\83N\82\AA\91\BD\8Fd\8C\9F\8Fo\82\B5\82\BD */
	ML_MULTIPLEX_DETECTION_MARK_ERROR,
	/**! \95K\97v\82\C8DLL\82\AA\8C\A9\82\A9\82\E7\82Ȃ\A2\82Ȃǎ\C0\8Ds\95s\89ȏ\F3\91\D4 */
	ML_NOT_EXECUTABLE,

	/**! \96\A2\89\F0\90͂̉摜\82\AA\97L\82\E8\83G\83\89\81[ */
	ML_THERE_UNANALYZED_IMAGE_ERROR,

	/**! \8F\E3\8BL\88ȊO\82̃G\83\89\81[ */
	ML_ERROR,
};

/**
*	@brief	\83\89\83C\83u\83\89\83\8A\81[\83G\83\89\81[\83R\81[\83h
*/
typedef	enum tagErrorCode	mlErrorCode;

#endif /* #ifndef MEASUREMENT_LIBRARY_H_ */
