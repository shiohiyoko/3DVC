#define __GLMETASEQ_C__
#include "GLMetaseq.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <GL/glext.h>

/*=========================================================================
  �ڤ��Υ�������ǤΤ�ͭ���ʥ����Х��ѿ���
  =========================================================================*/

static TEXTURE_POOL l_texPool[MAX_TEXTURE];		// �ƥ�������ס���
static int			l_texPoolnum;				// �ƥ�������ο�
static int			l_GLMetaseqInitialized = 0;	// ������ե饰


/*=========================================================================
  �ڴؿ������
  =========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

  void	endianConverter(void *addr,unsigned int size);
  void	TGAHeaderEndianConverter(	STR_TGA_HEAD *tgah );
  int		IsExtensionSupported( char* szTargetExtension );

  GLuint		mqoSetTexturePool(char *texfile, char *alpfile, unsigned char alpha );
  void		mqoClearTexturePool();
  GLubyte*	mqoLoadTextureEx(char *texfile,char *alpfile,int *tex_size,unsigned char alpha);
  int			mqoLoadFile(MQO_OBJECT *mqoobj,char *filename,double scale,unsigned char alpha);
  MQO_OBJECT*	mqoCreateList(int num);
  int			mqoCreateListObject( MQO_OBJECT *obj, int id, char *filename,double scale,unsigned char alpha);

  void mqoCallListObject(MQO_OBJECT object[],int num);
  void mqoClearObject(MQO_OBJECT object[],int from,int num);
  void mqoDeleteObject(MQO_OBJECT * object,int num);
  void mqoGetDirectory(const char *path_file, char *path_dir);
  void mqoSnormal(glPOINT3f A, glPOINT3f B, glPOINT3f C, glPOINT3f *normal);
  void mqoReadMaterial(FILE *fp, MQO_MATDATA M[]);
  void mqoReadVertex(FILE *fp, glPOINT3f V[]);
  int	 mqoReadBVertex(FILE *fp,glPOINT3f V[]);
  void mqoReadFace(FILE *fp, MQO_FACE F[]);
  void mqoReadObject(FILE *fp, MQO_OBJDATA *obj);

  void mqoMakeArray(MQO_MATERIAL *mat, int matpos,MQO_FACE F[], int fnum,glPOINT3f V[],
		    glPOINT3f N[], double facet, glCOLOR4f *mcol, double scale, unsigned char alpha );

  glPOINT3f *mqoVertexNormal(MQO_OBJDATA *obj);

  void mqoMakePolygon(MQO_OBJDATA *readObj, MQO_OBJECT *mqoobj,
		      glPOINT3f N[], MQO_MATDATA M[], int n_mat, double scale, unsigned char alpha);

  void mqoMakeObjectsEx(MQO_OBJECT *mqoobj, MQO_OBJDATA obj[], int n_obj, MQO_MATDATA M[],int n_mat,
			double scale,unsigned char alpha);

#ifdef __cplusplus
}
#endif


/*=========================================================================
  �ڴؿ���endianConverter
  �����ӡۥ���ǥ������Ѵ�
  �ڰ�����
  addr	���ɥ쥹
  size	������

  �����ۤ͡ʤ�
  =========================================================================*/

void endianConverter(void *addr,unsigned int size)
{
  unsigned int pos;
  char c;
  if ( size <= 1 ) return;
  for ( pos = 0; pos < size/2; pos++ ) {
    c = *(((char *)addr)+pos);
    *(((char *)addr)+pos) = *(((char *)addr)+(size-1 - pos));
    *(((char *)addr)+(size-1 - pos)) = c;
  }
}


/*=========================================================================
  �ڴؿ���TGAHeaderEndianConverter
  �����ӡ�TGA�Υإå��Υ���ǥ������Ѵ�
  �ڰ�����
  tgah	TGA�Υإå�

  �����ۤ͡ʤ�
  =========================================================================*/

void TGAHeaderEndianConverter(	STR_TGA_HEAD *tgah )
{
  endianConverter(&tgah->color_map_entry,sizeof(tgah->color_map_entry));
  endianConverter(&tgah->x,sizeof(tgah->x));
  endianConverter(&tgah->y,sizeof(tgah->y));
  endianConverter(&tgah->width,sizeof(tgah->width));
  endianConverter(&tgah->height,sizeof(tgah->height));
}


/*=========================================================================
  �ڴؿ���IsExtensionSupported
  �����ӡ�OpenGL�γ�ĥ��ǽ�����ݡ��Ȥ���Ƥ��뤫�ɤ���Ĵ�٤�
  �ڰ�����
  szTargetExtension	��ĥ��ǽ��̾��

  �����͡�1�����ݡ��Ȥ���Ƥ��롤0������Ƥ��ʤ�
  =========================================================================*/

int IsExtensionSupported( char* szTargetExtension )
{
  const unsigned char *pszExtensions = NULL;
  const unsigned char *pszStart;
  unsigned char *pszWhere, *pszTerminator;

  // Extension ��̾������������Ĵ�٤�(NULL������NG��
  pszWhere = (unsigned char *) strchr( szTargetExtension, ' ' );
  if ( pszWhere || *szTargetExtension == (char)NULL )
    return 0;

  // Extension ��ʸ������������
  pszExtensions = glGetString( GL_EXTENSIONS );

  // ʸ��������ɬ�פ� extension �����뤫Ĵ�٤�
  pszStart = pszExtensions;
  for (;;)
    {
      pszWhere = (unsigned char *) strstr( (const char *) pszStart, szTargetExtension );
      if ( !pszWhere )
	break;
      pszTerminator = pszWhere + strlen( szTargetExtension );
      if ( pszWhere == pszStart || *( pszWhere - 1 ) == ' ' )
	if ( *pszTerminator == ' ' || *pszTerminator == (char)NULL )
	  return 1;
      pszStart = pszTerminator;
    }
  return 0;
}


/*=========================================================================
  �ڴؿ���mqoInit
  �����ӡۥ᥿�������������ν����
  �ڰ����ۤʤ�
  �����ۤ͡ʤ�
  =========================================================================*/

void mqoInit(void)
{
  // �ƥ�������ס�������
  memset(l_texPool,0,sizeof(l_texPool));
  l_texPoolnum = 0;

  // ĺ���Хåե��Υ��ݡ��ȤΥ����å�
#if 0
  g_isVBOSupported = IsExtensionSupported("GL_ARB_vertex_buffer_object");
#else
  g_isVBOSupported = 0;
#endif  

#ifdef WIN32
  glGenBuffersARB = NULL;
  glBindBufferARB = NULL;
  glBufferDataARB = NULL;
  glDeleteBuffersARB = NULL;

  if ( g_isVBOSupported ) {
    // printf("OpenGL : ĺ���Хåե��򥵥ݡ��Ȥ��Ƥ���Τǻ��Ѥ��ޤ�\n");
    // GL �ؿ��Υݥ��󥿤��������
    glGenBuffersARB = (PFNGLGENBUFFERSARBPROC) wglGetProcAddress("glGenBuffersARB");
    glBindBufferARB = (PFNGLBINDBUFFERARBPROC) wglGetProcAddress("glBindBufferARB");
    glBufferDataARB = (PFNGLBUFFERDATAARBPROC) wglGetProcAddress("glBufferDataARB");
    glDeleteBuffersARB = (PFNGLDELETEBUFFERSARBPROC) wglGetProcAddress("glDeleteBuffersARB");
  }
#endif

  // ������ե饰
  l_GLMetaseqInitialized = 1;
}


/*=========================================================================
  �ڴؿ���mqoCleanup
  �����ӡۥ᥿�������������ν�λ����
  �ڰ����ۤʤ�
  �����ۤ͡ʤ�
  =========================================================================*/

void mqoCleanup(void)
{
  mqoClearTexturePool();	// �ƥ�������ס���Υ��ꥢ
}


/*=========================================================================
  �ڴؿ���mqoSetTexturePool
  �����ӡۥƥ�������ס���˥ƥ���������ɤ߹���
  �ڰ�����
  texfile		�ƥ�������ե�����̾
  alpfile		����ե��ե�����̾
  alpha		����ե�

  �����͡ۥƥ�������ID
  �ڻ��͡ۥƥ������㤬�ޤ��ɤ߹��ޤ�Ƥ��ʤ�����ɤ߹��ߡ��ƥ���������Ͽ
  ���Ǥ��ɤ߹��ޤ�Ƥ������Ͽ������Τ��֤�.
  =========================================================================*/

GLuint mqoSetTexturePool(char *texfile, char *alpfile, unsigned char alpha ) 
{
  int pos;
  GLubyte *image;

  for ( pos = 0; pos < l_texPoolnum; pos++ ) {
    if ( alpha != l_texPool[pos].alpha ) {
      continue;
    }
    if ( texfile != NULL ) {
      if ( strcmp(texfile,l_texPool[pos].texfile) != 0 ) {
	continue;
      }
    }
    if ( alpfile != NULL ) {
      if ( strcmp(alpfile,l_texPool[pos].alpfile) != 0 ) {
	continue;
      }
    }
    break;
  }
  if ( pos < l_texPoolnum ) { //���Ǥ��ɤ߹��ߺѤ�
    return  l_texPool[pos].texture_id;
  }
  if ( MAX_TEXTURE <= pos ) {
    printf("%s:mqoSetTexturePool �ƥ��������ɤ߹����ΰ���­\n",__FILE__);
    return -1;
  }
  image = mqoLoadTextureEx(texfile,alpfile,&l_texPool[pos].texsize,alpha);
  if ( image == NULL ) {
    return -1;
  }

  if ( texfile != NULL ) strncpy(l_texPool[pos].texfile,texfile,MAX_PATH);
  if ( alpfile != NULL ) strncpy(l_texPool[pos].alpfile,alpfile,MAX_PATH);
  l_texPool[pos].alpha = alpha;

  glPixelStorei(GL_UNPACK_ALIGNMENT,4);
  glPixelStorei(GL_PACK_ALIGNMENT,4);
  glGenTextures(1,&l_texPool[pos].texture_id);			// �ƥ������������
  glBindTexture(GL_TEXTURE_2D,l_texPool[pos].texture_id);	// �ƥ�������γ������

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, l_texPool[pos].texsize, l_texPool[pos].texsize,
	       0, GL_RGBA, GL_UNSIGNED_BYTE, image);
  l_texPoolnum = pos+1;

  //��Ͽ����С��ɤ߹�����Хåե�������
  free(image);
  glBindTexture(GL_TEXTURE_2D,0);	// �ǥե���ȥƥ�������γ������

  return l_texPool[pos].texture_id;
}


/*=========================================================================
  �ڴؿ���mqoClearTexturePool()
  �����ӡۥƥ�������ס���γ���
  �ڰ����ۤʤ�
  �����ۤ͡ʤ�
  =========================================================================*/

void mqoClearTexturePool()
{
  int pos;
  for ( pos = 0; pos < l_texPoolnum; pos++ ) {
    glDeleteTextures(1, &l_texPool[pos].texture_id);	// �ƥ�������������
  }

  memset(l_texPool,0,sizeof(l_texPool));
  l_texPoolnum = 0;
}


/*=========================================================================
  �ڴؿ���mqoLoadTextureEx
  �����ӡۥե����뤫��ƥ�������������������
  �ڰ�����
  texfile		�ե�����̾
  alpfile		����ե��ե�����̾
  tex_size	�ƥ�������Υ������ʰ��դ�Ĺ���ˤ��֤�

  �����͡ۥƥ�����������ؤΥݥ��󥿡ʼ��Ի���NULL��
  �ڻ��͡�24bit�ӥåȥޥåס������8,24,32bit�ԣǣ�
  �������ϡְ��դ�2��n����������פ˸���
  libjpeg,libpng�ʳ����饤�֥��ˤ�ͭ���JPEG,PNG���ɤ߹��߲�ǽ
  =========================================================================*/

GLubyte* mqoLoadTextureEx(char *texfile,char *alpfile,int *tex_size,unsigned char alpha)
{
  FILE *fp;
  size_t namelen;
  char ext[4];
  char wbuf[3];
  int isTGA;
  int isPNG;
  int isJPEG;
  int other;
  int	y,x,size;
  int fl;
  char *filename[2];
  int width[2];
  int sts;
  STR_TGA_HEAD tgah;
  GLubyte	*pImage, *pRead;

#if DEF_USE_LIBJPEG
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;
  JSAMPARRAY jpegimage;
#endif
#if DEF_USE_LIBPNG
  unsigned char **pngimage;
#if 0  
  unsigned long   pngwidth, pngheight;
#else
  png_uint_32	pngwidth, pngheight;
#endif
  int				pngdepth;
  int             color_type;
#endif

  filename[0] = texfile;
  filename[1] = alpfile;
  width[0] = -1;
  width[1] = -1;
  pImage = NULL;
  fp = NULL;
  sts = 0;
#if DEF_USE_LIBJPEG
  jpegimage = NULL;
#endif
#if DEF_USE_LIBPNG
  pngimage = NULL;
#endif
  size = - 1;
  for ( fl = 0; fl < 2; fl++ ) {//�ƥ��������fl=0    ����ե���fl=1
    if ( filename[fl] == NULL ) continue;
    namelen = strlen(filename[fl]);
    ext[0] = tolower(filename[fl][namelen-3]);
    ext[1] = tolower(filename[fl][namelen-2]);
    ext[2] = tolower(filename[fl][namelen-1]);
    ext[3] = 0x00;
    isTGA = (strcmp(ext,"tga")==0)?1:0;
    isPNG = (strcmp(ext,"png")==0)?1:0;
    isJPEG = (strcmp(ext,"jpg")==0)?1:0;
    /* */
    if ( (! isTGA) && (! isPNG) &&(! isJPEG) ) {
      filename[fl][namelen-3] = 'b';
      filename[fl][namelen-2] = 'm';
      filename[fl][namelen-1] = 'p';
    }
    /* */
    if ( fl == 1 ) { //����ե����ɤ߹��ߤϣԣǣ�or�УΣ�
      if ( ! (isTGA || isPNG) ) {
	printf("����ե��Υե�������б��Ǥ��ʤ���%s\n",filename[fl]);
	break;
      }
    }
    if ( fp != NULL ) fclose(fp);
    if ( (fp=fopen(filename[fl],"rb"))==NULL ) {
      printf("%s:�ƥ��������ɤ߹��ߥ��顼[%s]\n",__FILE__,filename[fl]);
      continue;
    }
    // �إå��Υ���
    if ( isTGA ) {
      fread(&tgah,sizeof(STR_TGA_HEAD),1,fp);
#if DEF_IS_LITTLE_ENDIAN
#else
      TGAHeaderEndianConverter(&tgah);
#endif
      size = width[fl] = tgah.width;
    }
    if ( isJPEG ) {
#if DEF_USE_LIBJPEG
      unsigned int i;
      cinfo.err = jpeg_std_error( &jerr );
      jpeg_create_decompress( &cinfo );	//�����Ѿ������
      jpeg_stdio_src( &cinfo, fp );		//�ɤ߹��ߥե��������
      jpeg_read_header( &cinfo, TRUE );	//jpeg�إå��ɤ߹���
      jpeg_start_decompress( &cinfo );	//���೫��

      if ( cinfo.out_color_components == 3 && cinfo.out_color_space == JCS_RGB ) {
	if ( jpegimage != NULL ) {
	  for (i = 0; i < cinfo.output_height; i++) free(jpegimage[i]);            // �ʲ����Ԥϣ����������������ޤ�
	  free(jpegimage);
	}
	//�ɤ߹��ߥǡ�������κ���
	jpegimage = (JSAMPARRAY)malloc( sizeof( JSAMPROW ) * cinfo.output_height );
	for ( i = 0; i < cinfo.output_height; i++ ) {
	  jpegimage[i] = (JSAMPROW)malloc( sizeof( JSAMPLE ) * cinfo.out_color_components * cinfo.output_width );
	}
	//����ǡ����ɤ߹���
	while( cinfo.output_scanline < cinfo.output_height ) {
	  jpeg_read_scanlines( &cinfo,
			       jpegimage + cinfo.output_scanline,
			       cinfo.output_height - cinfo.output_scanline
			       );
	}
	size = width[fl] = cinfo.output_width;
      }

      jpeg_finish_decompress( &cinfo );	//���ཪλ
      jpeg_destroy_decompress( &cinfo );	//�����Ѿ������
      if ( !(cinfo.out_color_components == 3 && cinfo.out_color_space == JCS_RGB) ) {
	printf("JPEG �б��Ǥ��ʤ��ե����ޥåȢ�%s\n",filename[fl]);
      }
#else
      printf("���Υƥ���������б��Ǥ��ʤ��ե����ޥåȢ�%s\n",filename[fl]);
      continue;
#endif
    }
    if ( isPNG ) {
#if DEF_USE_LIBPNG
      png_structp     png_ptr;
      png_infop       info_ptr;
      int             bit_depth, interlace_type;
      unsigned int             i;
      int j,k;
      png_ptr = png_create_read_struct(                       // png_ptr��¤�Τ���ݡ���������ޤ�
				       PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
      info_ptr = png_create_info_struct(png_ptr);             // info_ptr��¤�Τ���ݡ���������ޤ�
      png_init_io(png_ptr, fp);                               // libpng��fp���Τ餻�ޤ�
      png_read_info(png_ptr, info_ptr);                       // PNG�ե�����Υإå����ɤ߹��ߤޤ�
      png_get_IHDR(png_ptr, info_ptr, &pngwidth, &pngheight,        // IHDR����󥯾����������ޤ�
		   &bit_depth, &color_type, &interlace_type,
		   &j,&k);
      if ( pngimage != NULL ) {
	for (i = 0; i < pngheight; i++) free(pngimage[i]);            // �ʲ����Ԥϣ����������������ޤ�
	free(pngimage);
      }
      pngimage = (png_bytepp)malloc(pngheight * sizeof(png_bytep)); // �ʲ����Ԥϣ������������ݤ��ޤ�
      i = png_get_rowbytes(png_ptr, info_ptr);
      pngdepth = i / pngwidth;
      for (i = 0; i < pngheight; i++)
	pngimage[i] = (png_bytep)malloc(png_get_rowbytes(png_ptr, info_ptr));
      png_read_image(png_ptr, pngimage);                         // �����ǡ������ɤ߹��ߤޤ�

      png_destroy_read_struct(                                // ���Ĥι�¤�ΤΥ����������ޤ�
			      &png_ptr, &info_ptr, (png_infopp)NULL);
      size = width[fl] = pngwidth;
#else
      printf("���Υƥ���������б��Ǥ��ʤ��ե����ޥåȢ�%s\n",filename[fl]);
      continue;
#endif
    }
    if ( width[fl] == -1 ) {//�����ޤǤ��ƥ����������ꤵ��Ƥ��ʤ����ᡡ�ӥåȥޥå�
      fseek(fp,14+4,SEEK_SET);		// ����������Ǽ����Ƥ�����֤ޤǥ�����
      fread(&size,sizeof(int),1,fp);	// BiWidth�ξ����������
      fseek(fp,14+40,SEEK_SET);		// ���ǥǡ�������Ǽ����Ƥ�����֤ޤǥ�����
#if DEF_IS_LITTLE_ENDIAN
#else
      endianConverter(&size,sizeof(int));
#endif
      width[fl] = size;

    }
    if ( width[0] != -1 && width[1] != -1 ) {
      if ( width[0] != width[1] ) {
	sts = -1;
	break;
      }
    }
    if ( fl == 1 && isTGA ) { //����ե����ɤ߹��ߤϣԣǣ��Σ��ӥåȥ�Υ����򣳣��ӥåȥե�
      if ( !(
	     (tgah.depth == 8 && tgah.type == DEF_TGA_TYPE_MONO) ||
	     (tgah.depth == 32 && tgah.type == DEF_TGA_TYPE_FULL) 
	     ) ) {
	break;
      }
    }
    if ( fl == 1 && isPNG ) { //����ե����ɤ߹��ߤϣУΣǤΥȥ��롼���顼�ܥ���ե���򥰥졼��������ܥ���ե�
#if DEF_USE_LIBPNG
      if ( !(
	     (color_type== 6 ) ||
	     (color_type== 4 ) 
	     ) ) {
	break;
      }
#endif
    }

    // ����γ���
    if ( pImage == NULL ) {
      pImage = (GLubyte*)malloc(sizeof(unsigned char)*size*size*4);
    }
    if (pImage==NULL) return NULL;
    for (y=0; y<size; y++){
      pRead = pImage + (size-1-y)*4*size;
      for (x=0; x<size; x++) {
	other = 1;
	if ( fl == 0 ) {
	  if ( isJPEG ) {
#if DEF_USE_LIBJPEG
	    pRead[0]= jpegimage[size-1-y][x*3];
	    pRead[1]= jpegimage[size-1-y][x*3+1];
	    pRead[2]= jpegimage[size-1-y][x*3+2];
	    pRead[3] = alpha;				// A
	    other = 0;
#endif
	  }
	  if ( isPNG ) {
#if DEF_USE_LIBPNG
	    if ( color_type == 2 || color_type==6 ) { 
	      pRead[0]= pngimage[size-1-y][x*pngdepth];
	      pRead[1]= pngimage[size-1-y][x*pngdepth+1];
	      pRead[2]= pngimage[size-1-y][x*pngdepth+2];
	      pRead[3] = alpha;				// A
	      if ( color_type == 6 ) {
		pRead[3]= pngimage[size-1-y][x*pngdepth+3];
	      }
	    }
	    other = 0;
#endif
	  }
	  if ( other )  {
	    fread(&pRead[2],1,1,fp);	// B
	    fread(&pRead[1],1,1,fp);	// G	
	    fread(&pRead[0],1,1,fp);	// R
	    pRead[3] = alpha;				// A
	    if ( isTGA && tgah.depth == 32 ) {
	      fread(&pRead[3],1,1,fp);	// A
	      if ( alpha < pRead[3] ) pRead[3] = alpha;
	    }
	  }
	}
	else {
	  if ( isPNG ) {
#if DEF_USE_LIBPNG
	    if ( color_type == 6 ) { //�ȥ��롼���顼�ܥ���ե�
	      pRead[3]= pngimage[size-1-y][x*pngdepth+3];
	    }
	    if ( color_type == 4 ) { //���졼��������ܥ���ե�
	      pRead[3]= pngimage[size-1-y][x*pngdepth+1];
	    }
	    if ( alpha < pRead[3] ) pRead[3] = alpha;
#endif
	  }
	  if ( isTGA ) {
	    if ( tgah.depth == 32 ) { //����ʤ��ǡ������ɤ����Ф�
	      fread(wbuf,3,1,fp);	// BGR
	    }
	    fread(&pRead[3],1,1,fp);	// A
	    if ( alpha < pRead[3] ) pRead[3] = alpha;
	  }
	}
	pRead+=4;
      }
    }
    fclose(fp);
    fp = NULL;
  }
  if ( sts != 0 ) {
    if ( pImage != NULL ) free(pImage);
    if ( fp != NULL ) fclose(fp);
  }
#if DEF_USE_LIBPNG
  if ( pngimage != NULL ) {
    unsigned int uy;
    for (uy = 0; uy < pngheight; uy++) free(pngimage[uy]);            // �ʲ����Ԥϣ����������������ޤ�
    free(pngimage);
  }
#endif
#if DEF_USE_LIBJPEG
  if ( jpegimage != NULL ) {
    unsigned int uy;
    for (uy = 0; uy < cinfo.output_height; uy++) free(jpegimage[uy]);            // �ʲ����Ԥϣ����������������ޤ�
    free(jpegimage);
  }
#endif
  if ( size < 0 ) {
    if ( pImage != NULL ) free(pImage);
    pImage = NULL;
  }
  *tex_size = size;

  return pImage;
}


/*=========================================================================
  �ڴؿ���mqoLoadFile
  �����ӡۥ᥿���������ե�����(*.mqo)����ǡ������ɤ߹���
  �ڰ�����
  mqoobj		MQO���֥�������
  filename	�ե�����Υѥ�
  scale		����Ψ
  alpha		����ե�

  �����͡�������1 �� ���ԡ�0
  =========================================================================*/

int mqoLoadFile( MQO_OBJECT *mqoobj, char *filename, double scale, unsigned char alpha)
{
  FILE			*fp;
  MQO_OBJDATA		obj[MAX_OBJECT];
  MQO_MATDATA		*M = NULL;

  char	buf[SIZE_STR];		// ʸ�����ɤ߹��ߥХåե�
  char	path_dir[SIZE_STR];	// �ǥ��쥯�ȥ�Υѥ�
  char	path_tex[SIZE_STR];	// �ƥ�������ե�����Υѥ�
  char	path_alp[SIZE_STR];	// ����ե��ƥ�������ե�����Υѥ�
  int		n_mat = 0;			// �ޥƥꥢ���
  int		n_obj = 0;			// ���֥������ȿ�
  int		i;

  // Material��Object���ɤ߹���
  fp = fopen(filename,"rb");
  if (fp==NULL) return 0;

  mqoobj->alpha = alpha;
  memset(obj,0,sizeof(obj));

  i = 0;
  while ( !feof(fp) ) {
    fgets(buf,SIZE_STR,fp);

    // Material
    if (strstr(buf,"Material")) {
      sscanf(buf,"Material %d", &n_mat);
      M = (MQO_MATDATA*) calloc( n_mat, sizeof(MQO_MATDATA) );
      mqoReadMaterial(fp,M);
    }

    // Object
    if (strstr(buf,"Object")) {
      sscanf(buf,"Object %s", obj[i].objname);
      mqoReadObject(fp, &obj[i]);
      i++;
    }
  }
  n_obj = i;
  fclose(fp);

  // �ѥ��μ���
  mqoGetDirectory(filename, path_dir);

  // �ƥ����������Ͽ
  for (i=0; i<n_mat; i++) {
    if (M[i].useTex) {

      if (strstr(M[i].texFile,":")) {
	strcpy(path_tex, M[i].texFile);	// ���Хѥ��ξ��
      } else {
	sprintf(path_tex,"%s%s",path_dir,M[i].texFile);	// ���Хѥ��ξ��
      }

      if ( M[i].alpFile[0] != (char)0 ) {
	if (strstr(M[i].texFile,":")) {
	  strcpy(path_alp, M[i].alpFile);	// ���Хѥ��ξ��
	} else {
	  sprintf(path_alp,"%s%s",path_dir,M[i].alpFile);	// ���Хѥ��ξ��
	}
	M[i].texName = mqoSetTexturePool(path_tex,path_alp,alpha);
      }
      else {
	M[i].texName = mqoSetTexturePool(path_tex,NULL,alpha);
      }
    }
  }

  mqoMakeObjectsEx( mqoobj, obj, n_obj, M, n_mat, scale, alpha );

  // ���֥������ȤΥǡ����γ���
  for (i=0; i<n_obj; i++) {
    free(obj[i].V);
    free(obj[i].F);
  }

  // �ޥƥꥢ��γ���
  free(M);

  return 1;
}


/*=========================================================================
  �ڴؿ���mqoCreateList
  �����ӡ�MQO���֥������Ȥ��������ݤ���
  �ڰ�����num		MQO���֥������Ȥο�

  �����͡�MQO���֥�������
  =========================================================================*/

MQO_OBJECT* mqoCreateList(int num)
{
  MQO_OBJECT *obj;

  // ���������Ƥʤ��ä�������
  if ( ! l_GLMetaseqInitialized ) mqoInit();

  // �ΰ���ݤȽ����
  obj = (MQO_OBJECT *)malloc(sizeof(MQO_OBJECT)*num);
  memset(obj, 0, sizeof(MQO_OBJECT)*num);

  return obj;
}


/*=========================================================================
  �ڴؿ���mqoCreateListObject
  �����ӡۥ᥿���������ե�����(*.mqo)����MQO���֥�������������������

  �ڰ�����mqoobj		MQO���֥�������
  i			�ɤ߹������ֹ��i���ܤ�MQO�ե�������ɤ߹����
  filename	�ե�����Υѥ�
  scale		����Ψ
  alpha		����ե���������ΤΥ���ե��ͤ�����0��255�ˡ�

  �����͡ۥ��ơ��������顧�۾��������
  =========================================================================*/

int mqoCreateListObject(MQO_OBJECT *mqoobj, int i, char *filename, double scale, unsigned char alpha )
{
  int ret;
  ret = 0;
  if ( mqoobj == (MQO_OBJECT *)NULL ) return -1;
  if (! mqoLoadFile(&mqoobj[i], filename, scale, alpha)) ret = -1; 
  return ret;
}


/*=========================================================================
  �ڴؿ���mqoCallListObject
  �����ӡ�MQO���֥������Ȥ�OpenGL�β��̾�˸ƤӽФ�
  �ڰ�����
  mqoobj		MQO���֥�����������
  num			�����ֹ� (0����

  �����ۤ͡ʤ�
  =========================================================================*/

void mqoCallListObject(MQO_OBJECT mqoobj[],int num)
{

  MQO_INNER_OBJECT	*obj;
  MQO_MATERIAL		*mat;
  GLfloat				matenv[4];
  GLint				bindGL_TEXTURE_2D	= 0;
  GLboolean			isGL_TEXTURE_2D		= GL_FALSE;
  GLboolean			isGL_BLEND			= GL_FALSE;
  GLint				blendGL_SRC_ALPHA	= 0;
  GLint				intFrontFace;

  int		o, m, offset;
  double	dalpha;
  char	*base;

  if ( mqoobj == NULL) return;

  glPushMatrix();
  //�᥿������ĺ�����¤Ӥ�ɽ�̤���ߤƱ����
  glGetIntegerv(GL_FRONT_FACE,&intFrontFace);
  glFrontFace(GL_CW);
  dalpha = (double)mqoobj[num].alpha/(double)255;

  for ( o=0; o<mqoobj[num].objnum; o++ ) {	// �������֥������ȥ롼��

    obj = &mqoobj[num].obj[o];
    if ( ! obj->isVisible ) continue;
    glShadeModel(((obj->isShadingFlat))?GL_FLAT:GL_SMOOTH);

    for ( m = 0; m < obj->matnum; m++ ) {	//�ޥƥꥢ��롼��

      mat = &obj->mat[m];
      if ( mat->datanum == 0 ) continue;

      if ( mat->isValidMaterialInfo ) {	// �ޥƥꥢ��ξ�������
	memcpy(matenv,mat->dif,sizeof(matenv));
	matenv[3] *= dalpha;
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, matenv);
	memcpy(matenv,mat->amb,sizeof(matenv));
	matenv[3] *= dalpha;
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, matenv);
	memcpy(matenv,mat->spc,sizeof(matenv));
	matenv[3] *= dalpha;
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, matenv);
	memcpy(matenv,mat->emi,sizeof(matenv));
	matenv[3] *= dalpha;
	glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, matenv);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, mat->power);
      }

      if ( mat->isUseTexture) {	// �ƥ������㤬������
	glEnableClientState( GL_VERTEX_ARRAY );
	glEnableClientState( GL_NORMAL_ARRAY );
	glEnableClientState( GL_TEXTURE_COORD_ARRAY );

	isGL_TEXTURE_2D = glIsEnabled(GL_TEXTURE_2D);
	isGL_BLEND = glIsEnabled(GL_BLEND);
	glGetIntegerv(GL_TEXTURE_BINDING_2D,&bindGL_TEXTURE_2D);
	//					glGetIntegerv(GL_BLEND_SRC_ALPHA,&blendGL_SRC_ALPHA);

	glEnable(GL_TEXTURE_2D);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

	glBindTexture(GL_TEXTURE_2D,mat->texture_id);

	if ( g_isVBOSupported ) {	// ĺ���Хåե�����
	  base = (char *)NULL;	// ���ɥ쥹��NULL����Ƭ
#if 0
	  glBindBufferARB( GL_ARRAY_BUFFER_ARB, mat->VBO_id ); // ĺ���Хåե����ӤĤ���
#endif	  
	}
	else {
	  // ĺ������λ��ϡ����ɥ쥹�򤽤Τޤ������
	  base = (char *)mat->vertex_t[0].point;
	}

	// ĺ�����������
	offset = (int)( (char *)mat->vertex_t[0].point - (char *)mat->vertex_t[0].point );
	glVertexPointer( 3, GL_FLOAT, sizeof(VERTEX_TEXUSE) , base + offset );

	// �ƥ��������ɸ���������
	offset = (int)((char *)mat->vertex_t[0].uv-(char *)mat->vertex_t[0].point);
	glTexCoordPointer( 2, GL_FLOAT, sizeof(VERTEX_TEXUSE) , base + offset );

	// ˡ�����������
	offset = (int)((char *)mat->vertex_t[0].normal-(char *)mat->vertex_t[0].point);
	glNormalPointer( GL_FLOAT, sizeof(VERTEX_TEXUSE) , base+offset );

	// ������
	glColor4f(mat->color[0],mat->color[1],mat->color[2],mat->color[3]);

	// ����¹�
	glDrawArrays( GL_TRIANGLES, 0, mat->datanum );

	glBindTexture(GL_TEXTURE_2D,bindGL_TEXTURE_2D);
	if( isGL_BLEND == GL_FALSE ) glDisable(GL_BLEND);
	if( isGL_TEXTURE_2D == GL_FALSE ) glDisable(GL_TEXTURE_2D);

	if ( g_isVBOSupported ) {						// ĺ���Хåե�����
#if 0	  
	  glBindBufferARB( GL_ARRAY_BUFFER_ARB, 0 );	// ĺ���Хåե���ǥե���Ȥ�
#endif	  
	}

	glDisableClientState( GL_VERTEX_ARRAY );
	glDisableClientState( GL_NORMAL_ARRAY );
	glDisableClientState( GL_TEXTURE_COORD_ARRAY );
      }
      else {	// �ƥ������㤬�ʤ����

	glEnableClientState( GL_VERTEX_ARRAY );
	glEnableClientState( GL_NORMAL_ARRAY );
	//	glEnableClientState( GL_COLOR_ARRAY );

	isGL_BLEND = glIsEnabled(GL_BLEND);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

	if ( g_isVBOSupported ) {	// ĺ���Хåե�����
	  base = (char *)NULL;
#if 0	  
	  glBindBufferARB( GL_ARRAY_BUFFER_ARB, mat->VBO_id );
#endif	  
	}
	else {
	  base = (char *)mat->vertex_p[0].point;
	}

	// ĺ�����������
	offset = (int)((char *)mat->vertex_p[0].point-(char *)mat->vertex_p[0].point);
	glVertexPointer( 3, GL_FLOAT, sizeof(VERTEX_NOTEX) , base+offset );

	// ˡ�����������
	offset = (int)((char *)mat->vertex_p[0].normal-(char *)mat->vertex_p[0].point);
	glNormalPointer( GL_FLOAT, sizeof(VERTEX_NOTEX) , base+offset );

	// ������
	glColor4f(mat->color[0],mat->color[1],mat->color[2],mat->color[3]);
	//	offset = (int)((char *)mat->vertex_p[0].color-(char *)mat->vertex_p[0].point);
	//	glColorPointer(4,GL_FLOAT,sizeof(VERTEX_NOTEX),base+offset);

	// ����¹�
	glDrawArrays( GL_TRIANGLES, 0, mat->datanum );

	if( isGL_BLEND == GL_FALSE ) glDisable(GL_BLEND);
	if ( g_isVBOSupported ) {						// ĺ���Хåե�����
#if 0	  
	  glBindBufferARB( GL_ARRAY_BUFFER_ARB, 0 );	// ĺ���Хåե���ǥե���Ȥ�
#endif
	}

	//	glDisableClientState( GL_COLOR_ARRAY );
	glDisableClientState( GL_VERTEX_ARRAY );
	glDisableClientState( GL_NORMAL_ARRAY );

      }
    }
  }

  //�᥿������ĺ�����¤Ӥ�ɽ�̤���ߤƱ����ʸ�������ˤ�ɤ���
  glFrontFace(intFrontFace);
  glPopMatrix();
}


/*=========================================================================
  �ڴؿ���mqoGetDirectory
  �����ӡۥե�����̾��ޤ�ѥ�ʸ���󤫤�ǥ��쥯�ȥ�Υѥ��Τߤ���Ф���
  �ڰ�����
  *path_file	�ե�����̾��ޤ�ѥ�ʸ��������ϡ�
  *path_dir	�ե�����̾��������ѥ�ʸ����ʽ��ϡ�

  �����ۤ͡ʤ�
  �ڻ��͡��㡧
  "C:/data/file.bmp" �� "C:/data/"
  "data/file.mqo"    �� "data/"
  =========================================================================*/

void mqoGetDirectory(const char *path_file, char *path_dir)
{
  char *pStr;
  int len;

  pStr = (char*) MY_MAX( strrchr(path_file,'\\'), strrchr(path_file,'/') );
  len = MY_MAX((int)(pStr-path_file)+1,0);
  strncpy(path_dir,path_file,len);
  path_dir[len] = (char)0;
}


/*=========================================================================
  �ڴؿ���mqoSnormal
  �����ӡ�ˡ���٥��ȥ�����
  �ڰ�����
  A		3������ɸ�����A
  B		3������ɸ�����B
  C		3������ɸ�����C
  *normal	�٥��ȥ�BA�ȥ٥��ȥ�BC��ˡ���٥��ȥ�ʱ��ͤ�������

  �����ۤ͡ʤ�
  �ڻ��͡ۥ᥿���������ˤ������̤�������ĺ�����ֹ�ϡ�ɽ���̤��鸫��
  ���ײ��˵��Ҥ��Ƥ��롥�Ĥޤꡤĺ��A,B,C �����ä��Ȥ���
  ����٤�ˡ����BA��BC�γ��Ѥˤ�äƵ�����
  =========================================================================*/

void mqoSnormal(glPOINT3f A, glPOINT3f B, glPOINT3f C, glPOINT3f *normal)
{
  double norm;
  glPOINT3f vec0,vec1;

  // �٥��ȥ�BA
  vec0.x = A.x - B.x; 
  vec0.y = A.y - B.y;
  vec0.z = A.z - B.z;

  // �٥��ȥ�BC
  vec1.x = C.x - B.x;
  vec1.y = C.y - B.y;
  vec1.z = C.z - B.z;

  // ˡ���٥��ȥ�
  normal->x = vec0.y * vec1.z - vec0.z * vec1.y;
  normal->y = vec0.z * vec1.x - vec0.x * vec1.z;
  normal->z = vec0.x * vec1.y - vec0.y * vec1.x;

  // ������
  norm = normal->x * normal->x + normal->y * normal->y + normal->z * normal->z;
  norm = sqrt ( norm );

  normal->x /= norm;
  normal->y /= norm;
  normal->z /= norm;
}


/*=========================================================================
  �ڴؿ���mqoReadMaterial
  �����ӡۥޥƥꥢ�������ɤ߹���
  �ڰ�����
  fp		�ե�����ݥ���
  M		�ޥƥꥢ������

  �����ۤ͡ʤ�
  �ڻ��͡�mqoCreateModel(), mqoCreateSequence()�Υ��ִؿ���
  =========================================================================*/

void mqoReadMaterial(FILE *fp, MQO_MATDATA M[])
{
  GLfloat		dif, amb, emi, spc;
  glCOLOR4f	c;
  char		buf[SIZE_STR];
  char		*pStrEnd, *pStr;
  int			len;
  int			i = 0;

  while (1) {
    fgets(buf,SIZE_STR,fp);	// ���ɤ߹���
    if (strstr(buf,"}")) break;

    pStr = strstr(buf,"col(");	// ���̾�ɤ����Ф�
    sscanf( pStr,
	    "col(%f %f %f %f) dif (%f) amb(%f) emi(%f) spc(%f) power(%f)",
	    &c.r, &c.g, &c.b, &c.a, &dif, &amb, &emi, &spc, &M[i].power );

    // ĺ�����顼
    M[i].col = c;

    // �Ȼ���
    M[i].dif[0] = dif * c.r;
    M[i].dif[1] = dif * c.g;
    M[i].dif[2] = dif * c.b;
    M[i].dif[3] = c.a;

    // ���ϸ�
    M[i].amb[0] = amb * c.r;
    M[i].amb[1] = amb * c.g;
    M[i].amb[2] = amb * c.b;
    M[i].amb[3] = c.a;

    // ���ʾ���
    M[i].emi[0] = emi * c.r;
    M[i].emi[1] = emi * c.g;
    M[i].emi[2] = emi * c.b;
    M[i].emi[3] = c.a;

    // ȿ�͸�
    M[i].spc[0] = spc * c.r;
    M[i].spc[1] = spc * c.g;
    M[i].spc[2] = spc * c.b;
    M[i].spc[3] = c.a;
		
    // tex�����ͥޥåԥ�̾
    if ( (pStr = strstr(buf,"tex(")) != NULL ) {
      M[i].useTex = TRUE;

      pStrEnd = strstr(pStr,")")-1;
      len = pStrEnd - (pStr+5);
      strncpy(M[i].texFile,pStr+5,len);
      M[i].texFile[len] = (char)0;
      if ( (pStr = strstr(buf,"aplane(")) != NULL ) {
	pStrEnd = strstr(pStr,")")-1;
	len = pStrEnd - (pStr+8);
	strncpy(M[i].alpFile,pStr+8,len);
	M[i].alpFile[len] = (char)0;
      }
      else {
	M[i].alpFile[0] = (char)0;
      }

    } else {
      M[i].useTex = FALSE;
      M[i].texFile[0] = (char)0;
      M[i].alpFile[0] = (char)0;
    }

    i++;
  }

}


/*=========================================================================
  �ڴؿ���mqoReadVertex
  �����ӡ�ĺ��������ɤ߹���
  �ڰ�����
  fp		���ߥ����ץ󤷤Ƥ���᥿���������ե�����Υե�����ݥ���
  V		ĺ�����Ǽ��������
		
  �����ۤ͡ʤ�
  �ڻ��͡�mqoReadObject()�Υ��ִؿ�
  =========================================================================*/

void mqoReadVertex(FILE *fp, glPOINT3f V[])
{
  char buf[SIZE_STR];
  int  i=0;

  while (1) {
    fgets(buf,SIZE_STR,fp);
    if (strstr(buf,"}")) break;
    sscanf(buf,"%f %f %f",&V[i].x,&V[i].y,&V[i].z);
    i++;
  }
}


/*=========================================================================
  �ڴؿ���mqoReadBVertex
  �����ӡۥХ��ʥ������ĺ��������ɤ߹���
  �ڰ�����
  fp		���ߥ����ץ󤷤Ƥ���᥿���������ե�����Υե�����ݥ���
  V		ĺ�����Ǽ��������

  �����͡�ĺ����
  �ڻ��͡�mqoReadObject()�Υ��ִؿ�
  =========================================================================*/

int mqoReadBVertex(FILE *fp, glPOINT3f V[])
{
  int n_vertex,i;
  float *wf;
  int size;
  char cw[256];
  char *pStr;

  fgets(cw,sizeof(cw),fp);
  if ( (pStr = strstr(cw,"Vector")) != NULL ) {
    sscanf(pStr,"Vector %d [%d]",&n_vertex,&size);	// ĺ�������ǡ������������ɤ߹���
  }
  else {
    return -1;
  }
  //MQO�ե�����ΥХ��ʥ�ĺ���ǡ�����intel�����ʥ�ȥ륨�ǥ������
  wf = (float *)malloc(size);
  fread(wf,size,1,fp);
  for ( i = 0; i < n_vertex; i++ ) {
    V[i].x = wf[i*3+0];
    V[i].y = wf[i*3+1];
    V[i].z = wf[i*3+2];
#if DEF_IS_LITTLE_ENDIAN
#else
    endianConverter((void *)&V[i].x,sizeof(V[i].x));
    endianConverter(&V[i].y,sizeof(V[i].y));
    endianConverter(&V[i].z,sizeof(V[i].z));
#endif
  }
  free(wf);

  // "}"�ޤ��ɤ����Ф�
  {
    char buf[SIZE_STR];
    while (1) {
      fgets(buf,SIZE_STR,fp);
      if (strstr(buf,"}")) break;
    }
  }

  return n_vertex;
}


/*=========================================================================
  �ڴؿ���mqoReadFace
  �����ӡ��̾�����ɤ߹���
  �ڰ�����
  fp		�ե�����ݥ���
  F		������
		
  �����ۤ͡ʤ�
  �ڻ��͡�mqoReadObject()�Υ��ִؿ�
  =========================================================================*/

void mqoReadFace(FILE *fp, MQO_FACE F[])
{
  char buf[SIZE_STR];
  char *pStr;
  int  i=0;

  while (1) {
    fgets(buf,SIZE_STR,fp);
    if (strstr(buf,"}")) break;

    // �̤�������ĺ����
    sscanf(buf,"%d",&F[i].n);

    // ĺ��(V)���ɤ߹���
    if ( (pStr = strstr(buf,"V(")) != NULL ) {
      switch (F[i].n) {
      case 3:
	//�᥿������ĺ�����¤Ӥ�ɽ�̤���ߤƱ����
	//�ɤ߹��߻����¤��ؤ�����ˡ�⤢�롣���ɡ�ɽ�̤������
	//glFrontFace���Ѥ���ۤ������ޡ��ȡ�
	sscanf(pStr,"V(%d %d %d)",&F[i].v[0],&F[i].v[1],&F[i].v[2]);
	//					sscanf(pStr,"V(%d %d %d)",&F[i].v[2],&F[i].v[1],&F[i].v[0]);
	break;
      case 4:
	sscanf(pStr,"V(%d %d %d %d)",&F[i].v[0],&F[i].v[1],&F[i].v[2],&F[i].v[3]);
	//					sscanf(pStr,"V(%d %d %d %d)",&F[i].v[3],&F[i].v[2],&F[i].v[1],&F[i].v[0]);
	break;
      default:
	break;
      }		
    }

    // �ޥƥꥢ��(M)���ɤ߹���
    F[i].m = 0;
    if ( (pStr = strstr(buf,"M(")) != NULL ) {
      sscanf(pStr,"M(%d)",&F[i].m);
    }
    else { // �ޥƥꥢ�뤬���ꤵ��Ƥ��ʤ���
      F[i].m = -1;
    }

    // UV�ޥå�(UV)���ɤ߹���
    if ( (pStr = strstr(buf,"UV(")) != NULL ) {
      switch (F[i].n) {
      case 3:	// ĺ����3
	sscanf(pStr,"UV(%f %f %f %f %f %f)",
	       &F[i].uv[0].x, &F[i].uv[0].y,
	       &F[i].uv[1].x, &F[i].uv[1].y,
	       &F[i].uv[2].x, &F[i].uv[2].y
	       );
	break;

      case 4:	// ĺ����4
	sscanf(pStr,"UV(%f %f %f %f %f %f %f %f)",
	       &F[i].uv[0].x, &F[i].uv[0].y,
	       &F[i].uv[1].x, &F[i].uv[1].y,
	       &F[i].uv[2].x, &F[i].uv[2].y,
	       &F[i].uv[3].x, &F[i].uv[3].y
	       );
	break;
      default:
	break;
      }		
    }

    i++;
  }

}


/*=========================================================================
  �ڴؿ���mqoReadObject
  �����ӡۥ��֥������Ⱦ�����ɤ߹���
  �ڰ�����
  fp		�ե�����ݥ���
  obj		���֥������Ⱦ���

  �����ۤ͡ʤ�
  �ڻ��ۤ͡��δؿ��ǣ��ĤΥ��֥������Ⱦ����ɤ߹��ޤ�롥
  =========================================================================*/

void mqoReadObject(FILE *fp, MQO_OBJDATA *obj)
{
  char buf[SIZE_STR];

  while (1) {
    fgets(buf,SIZE_STR,fp);
    if (strstr(buf,"}")) break;

    // visible
    if (strstr(buf,"visible ")) {
      sscanf(buf," visible %d", &obj->visible);
    }

    // shading
    if (strstr(buf,"shading ")) {
      sscanf(buf," shading %d", &obj->shading);
    }

    // facet
    if (strstr(buf,"facet ")) {
      sscanf(buf," facet %f", &obj->facet);
    }

    // vertex
    if (strstr(buf,"vertex ")) {
      sscanf(buf," vertex %d", &obj->n_vertex);
      obj->V = (glPOINT3f*) calloc( obj->n_vertex, sizeof(glPOINT3f) );
      mqoReadVertex(fp, obj->V);
    }
    // BVertex
    if (strstr(buf,"BVertex"))	{
      sscanf(buf," BVertex %d", &obj->n_vertex);
      obj->V = (glPOINT3f*) calloc( obj->n_vertex, sizeof(glPOINT3f) );
      mqoReadBVertex(fp,obj->V);
    }

    // face
    if (strstr(buf,"face ")) {
      sscanf(buf," face %d", &obj->n_face);
      obj->F = (MQO_FACE*) calloc( obj->n_face, sizeof(MQO_FACE) );
      mqoReadFace(fp, obj->F);
    }

  }

}


/*=========================================================================
  �ڴؿ���mqoMakeArray
  �����ӡ�ĺ������κ���
  �ڰ�����
  mat		�ޥƥꥢ��ʤ������ĺ���ǡ�����ޤ��
  matpos	����ֹ�
  F		��
  fnum	�̿�
  V		ĺ��
  N		ˡ��
  facet	���ࡼ���󥰳�
  mcol	��
  scale	��������
  alpha	����ե�

  �����ۤ͡ʤ�
  �ڻ��͡�ĺ������Ϥ��٤ƻ��Ѥˤ���Τǡ��ͳѤϻ��ѣ�����ʬ��
  0  3      0    0  3
  ��   ������������
  1  2     1  2   2  
  =========================================================================*/

void mqoMakeArray( MQO_MATERIAL *mat, int matpos, MQO_FACE F[], int fnum,glPOINT3f V[],
		   glPOINT3f N[], double facet, glCOLOR4f *mcol, double scale, unsigned char alpha )
{
  int f;
  int i;
  int dpos;
  double s;
  glPOINT3f normal;	// ˡ���٥��ȥ�
	
  dpos = 0;
  mat->color[0] = mcol->r;
  mat->color[1] = mcol->g;
  mat->color[2] = mcol->b;
  mat->color[3] = mcol->a;
  if ( mat->isUseTexture ) {
    for ( f = 0; f < fnum; f++ ){
      if ( F[f].m != matpos ) continue;
      if ( F[f].n == 3 ) {
	mqoSnormal(V[F[f].v[0]],V[F[f].v[1]],V[F[f].v[2]],&normal);	// ˡ���٥��ȥ��׻�
	for ( i = 0; i < 3; i++ ) {
	  mat->vertex_t[dpos].point[0] = V[F[f].v[i]].x*scale;
	  mat->vertex_t[dpos].point[1] = V[F[f].v[i]].y*scale;
	  mat->vertex_t[dpos].point[2] = V[F[f].v[i]].z*scale;
	  mat->vertex_t[dpos].uv[0] = F[f].uv[i].x;
	  mat->vertex_t[dpos].uv[1] = F[f].uv[i].y;
	  s = acos(normal.x*N[F[f].v[i]].x + normal.y*N[F[f].v[i]].y + normal.z*N[F[f].v[i]].z);
	  if ( facet < s ) {
	    // ���ࡼ���󥰳ѡ����ĺ��ˡ������ˡ���γ��١ˤΤȤ�����ˡ����ĺ��ˡ���Ȥ���
	    mat->vertex_t[dpos].normal[0] = normal.x;
	    mat->vertex_t[dpos].normal[1] = normal.y;
	    mat->vertex_t[dpos].normal[2] = normal.z;
	  }
	  else {
	    mat->vertex_t[dpos].normal[0] = N[F[f].v[i]].x;
	    mat->vertex_t[dpos].normal[1] = N[F[f].v[i]].y;
	    mat->vertex_t[dpos].normal[2] = N[F[f].v[i]].z;
	  }
	  dpos++;
	}
      }
      //��ĺ���ʻͳѡˤϣ�ĺ���ʻ��ѡˣ�����ʬ��
      if ( F[f].n == 4 ) {
	mqoSnormal(V[F[f].v[0]],V[F[f].v[1]],V[F[f].v[2]],&normal);	// ˡ���٥��ȥ��׻�
	for ( i = 0; i < 4; i++ ) {
	  if ( i == 3 ) continue;
	  mat->vertex_t[dpos].point[0] = V[F[f].v[i]].x*scale;
	  mat->vertex_t[dpos].point[1] = V[F[f].v[i]].y*scale;
	  mat->vertex_t[dpos].point[2] = V[F[f].v[i]].z*scale;
	  mat->vertex_t[dpos].uv[0] = F[f].uv[i].x;
	  mat->vertex_t[dpos].uv[1] = F[f].uv[i].y;
	  s = acos(normal.x*N[F[f].v[i]].x + normal.y*N[F[f].v[i]].y + normal.z*N[F[f].v[i]].z);
	  if ( facet < s ) {
	    mat->vertex_t[dpos].normal[0] = normal.x;
	    mat->vertex_t[dpos].normal[1] = normal.y;
	    mat->vertex_t[dpos].normal[2] = normal.z;
	  }
	  else {
	    mat->vertex_t[dpos].normal[0] = N[F[f].v[i]].x;
	    mat->vertex_t[dpos].normal[1] = N[F[f].v[i]].y;
	    mat->vertex_t[dpos].normal[2] = N[F[f].v[i]].z;
	  }
	  dpos++;
	}
	mqoSnormal(V[F[f].v[0]],V[F[f].v[2]],V[F[f].v[3]],&normal);	// ˡ���٥��ȥ��׻�
	for ( i = 0; i < 4; i++ ) {
	  if ( i == 1 ) continue;
	  mat->vertex_t[dpos].point[0] = V[F[f].v[i]].x*scale;
	  mat->vertex_t[dpos].point[1] = V[F[f].v[i]].y*scale;
	  mat->vertex_t[dpos].point[2] = V[F[f].v[i]].z*scale;
	  mat->vertex_t[dpos].uv[0] = F[f].uv[i].x;
	  mat->vertex_t[dpos].uv[1] = F[f].uv[i].y;
	  s = acos(normal.x*N[F[f].v[i]].x + normal.y*N[F[f].v[i]].y + normal.z*N[F[f].v[i]].z);
	  if ( facet < s ) {
	    mat->vertex_t[dpos].normal[0] = normal.x;
	    mat->vertex_t[dpos].normal[1] = normal.y;
	    mat->vertex_t[dpos].normal[2] = normal.z;
	  }
	  else {
	    mat->vertex_t[dpos].normal[0] = N[F[f].v[i]].x;
	    mat->vertex_t[dpos].normal[1] = N[F[f].v[i]].y;
	    mat->vertex_t[dpos].normal[2] = N[F[f].v[i]].z;
	  }
	  dpos++;
	}
      }
    }
  }
  else {
    if ( alpha != 255 ) {
      mat->color[3] = (double)alpha/(double)255;
    }
    for ( f = 0; f < fnum; f++ ){
      if ( F[f].m != matpos ) continue;
      if ( F[f].n == 3 ) {
	mqoSnormal(V[F[f].v[0]],V[F[f].v[1]],V[F[f].v[2]],&normal);		// ˡ���٥��ȥ��׻�
	for ( i = 0; i < 3; i++ ) {
	  mat->vertex_p[dpos].point[0] = V[F[f].v[i]].x*scale;
	  mat->vertex_p[dpos].point[1] = V[F[f].v[i]].y*scale;
	  mat->vertex_p[dpos].point[2] = V[F[f].v[i]].z*scale;
	  mat->vertex_p[dpos].normal[0] = normal.x;
	  mat->vertex_p[dpos].normal[1] = normal.y;
	  mat->vertex_p[dpos].normal[2] = normal.z;
	  s = acos(normal.x*N[F[f].v[i]].x + normal.y*N[F[f].v[i]].y + normal.z*N[F[f].v[i]].z);
	  if ( facet < s ) {
	    mat->vertex_p[dpos].normal[0] = normal.x;
	    mat->vertex_p[dpos].normal[1] = normal.y;
	    mat->vertex_p[dpos].normal[2] = normal.z;
	  }
	  else {
	    mat->vertex_p[dpos].normal[0] = N[F[f].v[i]].x;
	    mat->vertex_p[dpos].normal[1] = N[F[f].v[i]].y;
	    mat->vertex_p[dpos].normal[2] = N[F[f].v[i]].z;
	  }
	  dpos++;
	}
      }
      //��ĺ���ʻͳѡˤϣ�ĺ���ʻ��ѡˣ�����ʬ��
      if ( F[f].n == 4 ) {
	mqoSnormal(V[F[f].v[0]],V[F[f].v[1]],V[F[f].v[2]],&normal);		// ˡ���٥��ȥ��׻�
	for ( i = 0; i < 4; i++ ) {
	  if ( i == 3 ) continue;
	  mat->vertex_p[dpos].point[0] = V[F[f].v[i]].x*scale;
	  mat->vertex_p[dpos].point[1] = V[F[f].v[i]].y*scale;
	  mat->vertex_p[dpos].point[2] = V[F[f].v[i]].z*scale;
	  mat->vertex_p[dpos].normal[0] = normal.x;
	  mat->vertex_p[dpos].normal[1] = normal.y;
	  mat->vertex_p[dpos].normal[2] = normal.z;
	  s = acos(normal.x*N[F[f].v[i]].x + normal.y*N[F[f].v[i]].y + normal.z*N[F[f].v[i]].z);
	  if ( facet < s ) {
	    mat->vertex_p[dpos].normal[0] = normal.x;
	    mat->vertex_p[dpos].normal[1] = normal.y;
	    mat->vertex_p[dpos].normal[2] = normal.z;
	  }
	  else {
	    mat->vertex_p[dpos].normal[0] = N[F[f].v[i]].x;
	    mat->vertex_p[dpos].normal[1] = N[F[f].v[i]].y;
	    mat->vertex_p[dpos].normal[2] = N[F[f].v[i]].z;
	  }
	  dpos++;
	}
	mqoSnormal(V[F[f].v[0]],V[F[f].v[2]],V[F[f].v[3]],&normal);		// ˡ���٥��ȥ��׻�
	for ( i = 0; i < 4; i++ ) {
	  if ( i == 1 ) continue;
	  mat->vertex_p[dpos].point[0] = V[F[f].v[i]].x*scale;
	  mat->vertex_p[dpos].point[1] = V[F[f].v[i]].y*scale;
	  mat->vertex_p[dpos].point[2] = V[F[f].v[i]].z*scale;
	  mat->vertex_p[dpos].normal[0] = normal.x;
	  mat->vertex_p[dpos].normal[1] = normal.y;
	  mat->vertex_p[dpos].normal[2] = normal.z;
	  s = acos(normal.x*N[F[f].v[i]].x + normal.y*N[F[f].v[i]].y + normal.z*N[F[f].v[i]].z);
	  if ( facet < s ) {
	    mat->vertex_p[dpos].normal[0] = normal.x;
	    mat->vertex_p[dpos].normal[1] = normal.y;
	    mat->vertex_p[dpos].normal[2] = normal.z;
	  }
	  else {
	    mat->vertex_p[dpos].normal[0] = N[F[f].v[i]].x;
	    mat->vertex_p[dpos].normal[1] = N[F[f].v[i]].y;
	    mat->vertex_p[dpos].normal[2] = N[F[f].v[i]].z;
	  }
	  dpos++;
	}
      }
    }
  }
}


/*=========================================================================
  �ڴؿ���mqoVertexNormal
  �����ӡ�ĺ��ˡ���η׻�
  �ڰ�����
  obj			���֥������Ⱦ���

  �����͡�ˡ������
  �ڻ��ۣ͡�ĺ�����̤ϻ��ѷ���ʬ�䤷�Ʒ׻�
  ����ͤ�ɬ���ƤӽФ����ǲ�����free�ˤ��뤳�ȡ�
  =========================================================================*/

glPOINT3f * mqoVertexNormal(MQO_OBJDATA *obj)
{
  int f;
  int v;
  int i;
  double len;
  glPOINT3f fnormal;	// ��ˡ���٥��ȥ�
  MQO_FACE *F;
  glPOINT3f *V;
  glPOINT3f *ret;
  F = obj->F;
  V = obj->V;
  ret = (glPOINT3f *)calloc(obj->n_vertex,sizeof(glPOINT3f));
  //�̤�ˡ����ĺ����­������
  for ( f = 0; f < obj->n_face; f++ ) {
    if ( obj->F[f].n == 3 ) {
      mqoSnormal(V[F[f].v[0]],V[F[f].v[1]],V[F[f].v[2]],&fnormal);
      for ( i = 0; i < 3; i++ ) {
	ret[F[f].v[i]].x += fnormal.x;
	ret[F[f].v[i]].y += fnormal.y;
	ret[F[f].v[i]].z += fnormal.z;
      }
    }
    if ( obj->F[f].n == 4 ) {
      mqoSnormal(V[F[f].v[0]],V[F[f].v[1]],V[F[f].v[2]],&fnormal);
      for ( i = 0; i < 4; i++ ) {
	if ( i == 3 ) continue;
	ret[F[f].v[i]].x += fnormal.x;
	ret[F[f].v[i]].y += fnormal.y;
	ret[F[f].v[i]].z += fnormal.z;
      }
      mqoSnormal(V[F[f].v[0]],V[F[f].v[2]],V[F[f].v[3]],&fnormal);
      for ( i = 0; i < 4; i++ ) {
	if ( i == 1 ) continue;
	ret[F[f].v[i]].x += fnormal.x;
	ret[F[f].v[i]].y += fnormal.y;
	ret[F[f].v[i]].z += fnormal.z;
      }
    }
  }
  //������
  for ( v = 0; v < obj->n_vertex; v++ ) {
    if ( ret[v].x == 0 && ret[v].y == 0 && ret[v].z == 0 ) {
      //�̤˻Ȥ��Ƥʤ���
      continue;
    }
    len = sqrt(ret[v].x*ret[v].x + ret[v].y*ret[v].y + ret[v].z*ret[v].z);
    if ( len != 0 ) {
      ret[v].x = ret[v].x/len;
      ret[v].y = ret[v].y/len;
      ret[v].z = ret[v].z/len;
    }
  }

  return ret;
}

/*=========================================================================
  �ڴؿ���mqoMakePolygon
  �����ӡۥݥꥴ�������
  �ڰ�����
  readObj		�ɤ߹�������֥������Ⱦ���
  mqoobj		MQO���֥������� 
  N[]			ˡ������
  M[]			�ޥƥꥢ������
  n_mat		�ޥƥꥢ���
  scale		��������
  alpha		����ե�

  �����ۤ͡ʤ�
  =========================================================================*/

void mqoMakePolygon(MQO_OBJDATA *readObj, MQO_OBJECT *mqoobj,
		    glPOINT3f N[], MQO_MATDATA M[], int n_mat, double scale, unsigned char alpha)
{

  MQO_INNER_OBJECT		*setObj;
  MQO_MATERIAL			*material;
  glCOLOR4f				defcol;
  glCOLOR4f				*pcol;
  int						f, m, *mat_vnum;
  int						fnum;
  MQO_FACE				*F;
  glPOINT3f				*V;
  double					facet;


  setObj = &mqoobj->obj[mqoobj->objnum];
  strcpy(setObj->objname,readObj->objname);
  setObj->isVisible = readObj->visible;
  setObj->isShadingFlat = (readObj->shading == 0);
  F = readObj->F;
  fnum = readObj->n_face;
  V = readObj->V;
  facet = readObj->facet;

  // face����ǤΥޥƥꥢ�����ĺ���ο�
  // M=NULL�ΤȤ���F[].m = 0 �����äƤ���
  if ( M == NULL ) n_mat = 1;

  mat_vnum = (int *)malloc(sizeof(int)*n_mat);
  memset(mat_vnum,0,sizeof(int)*n_mat);

  for ( f = 0; f < fnum; f++ ){
    if( F[f].m < 0 ) continue; // �ޥƥꥢ�뤬���ꤵ��Ƥ��ʤ���
    if ( F[f].n == 3 ) {
      mat_vnum[F[f].m] += 3;
    }
    if ( F[f].n == 4 ) {
      //��ĺ���ʻͳѡˤϣ�ĺ���ʻ��ѡˣ�����ʬ��
      //  0  3      0    0  3
      //   ��   ������������
      //  1  2     1  2   2
      // ��ĺ����ʿ�̥ǡ�����
      // ��ĺ����ʿ�̥ǡ���������
      mat_vnum[F[f].m] += 3*2;
    }
    if ( setObj->matnum < F[f].m+1 ) setObj->matnum = F[f].m+1;
  }

  // �ޥƥꥢ���̤�ĺ��������������
  setObj->mat = (MQO_MATERIAL *)malloc(sizeof(MQO_MATERIAL)*setObj->matnum);
  memset(setObj->mat,0,sizeof(MQO_MATERIAL)*setObj->matnum);

  for ( m = 0; m < setObj->matnum; m++ ) {
    material = &setObj->mat[m];
    material->datanum = mat_vnum[m];
    material->isValidMaterialInfo = (M != NULL);

    if ( mat_vnum[m] <= 0 ) continue;
    if ( material->isValidMaterialInfo ) {
      memcpy(material->dif,M[m].dif,sizeof(material->dif));
      memcpy(material->amb,M[m].amb,sizeof(material->amb));
      memcpy(material->spc,M[m].spc,sizeof(material->spc));
      memcpy(material->emi,M[m].emi,sizeof(material->emi));
      material->power = M[m].power;
      material->isUseTexture = M[m].useTex;
      pcol = &M[m].col;
    }
    else {
      defcol.r = 1.0;
      defcol.g = 1.0;
      defcol.b = 1.0;
      defcol.a = 1.0;
      material->isUseTexture = 0;
      pcol = &defcol;
    }
    if ( material->isUseTexture ) {
      material->vertex_t = (VERTEX_TEXUSE *)calloc(material->datanum,sizeof(VERTEX_TEXUSE));
      material->texture_id = M[m].texName;
    }
    else {
      material->vertex_p = (VERTEX_NOTEX *)calloc(material->datanum,sizeof(VERTEX_NOTEX));
    }
    mqoMakeArray(material,m,F,fnum,V,N,facet,pcol,scale,alpha);
    if (g_isVBOSupported) {
      if ( material->isUseTexture ) {
#if 0	
	glGenBuffersARB( 1, &material->VBO_id );
	glBindBufferARB( GL_ARRAY_BUFFER_ARB, material->VBO_id  );
	glBufferDataARB( GL_ARRAY_BUFFER_ARB, material->datanum*sizeof(VERTEX_TEXUSE), material->vertex_t, GL_STATIC_DRAW_ARB );
#endif	
      }
      else {
#if 0	
	glGenBuffersARB( 1, &material->VBO_id );
	glBindBufferARB( GL_ARRAY_BUFFER_ARB, material->VBO_id  );
	glBufferDataARB( GL_ARRAY_BUFFER_ARB, material->datanum*sizeof(VERTEX_NOTEX), material->vertex_p, GL_STATIC_DRAW_ARB );
#endif	
      }
    }
  }
  mqoobj->objnum++;
  if ( MAX_OBJECT <= mqoobj->objnum ) {
    printf("MQO�ե������ɤ߹��ߡ������祪�֥������ȿ���Ķ���ޤ���[%d]\n",mqoobj->objnum);
    mqoobj->objnum = MAX_OBJECT-1;
  }

  free(mat_vnum);

}


/*=========================================================================
  �ڴؿ���mqoMakeObjectsEx
  �����ӡۥ��֥������ȤΥǡ�������ݥꥴ���ǥ���������
  �ڰ�����
  mqoobj	MQO���֥�������
  obj		���֥�����������
  n_obj	���֥������ȤθĿ�
  M		�ޥƥꥢ������
  n_mat	�ޥƥꥢ��θĿ�
  scale	����Ψ
  alpha	����ե�

  �����ۤ͡ʤ�
  =========================================================================*/

void mqoMakeObjectsEx( MQO_OBJECT *mqoobj, MQO_OBJDATA obj[], int n_obj, MQO_MATDATA M[],int n_mat,
		       double scale,unsigned char alpha)
{
  int i;
  glPOINT3f *N;
  for (i=0; i<n_obj; i++) {
    N = mqoVertexNormal(&obj[i]);
    mqoMakePolygon(
		   &obj[i],
		   mqoobj,
		   N,
		   M,n_mat,
		   scale,
		   alpha);
    free(N);
  }
}


/*=========================================================================
  �ڴؿ���mqoCreateModel
  �����ӡ�MQO�ե����뤫��MQO��ǥ���������
  �ڰ�����
  filename	MQO�ե�����
  scale		����Ψ��1.0�Ǥ��Τޤޡ�

  �����͡�MQO_MODEL��MQO��ǥ��
  =========================================================================*/

MQO_MODEL mqoCreateModel(char *filename, double scale)
{
  MQO_MODEL ret;
  ret = mqoCreateList(1);
  if ( mqoCreateListObject(ret,1-1,filename,scale,(unsigned char)255) < 0 ) {
    mqoDeleteObject(ret,1);
    ret = NULL;
  }
  return ret;
}


/*=========================================================================
  �ڴؿ���mqoCreateSequenceEx
  �����ӡ�Ϣ�֤�MQO�ե����뤫��MQO�������󥹤��������
  �ڰ�����
  format		�ե�����̾�ν�
  n_file		�ե������
  scale		����Ψ��1.0�Ǥ��Τޤޡ�
  fade_inout	0:���Τޤޡ������ե����ɥ����顧�ե����ɥ�����
  �����ͤϸ��̤򤫤���ե졼���
  alpha		����ե�

  �����͡�MQO_SEQUENCE��MQO�������󥹡�
  �����͡�Ϣ�֤�0���鳫��
  =========================================================================*/

MQO_SEQUENCE mqoCreateSequenceEx(const char *format, int n_file, double scale,
				 int fade_inout, unsigned char alpha)
{
  MQO_SEQUENCE retSeq;
  int iret;
  int seq;
  char filename[SIZE_STR];
  short setAlpha;
  short calAlpha;
  int frames;

  retSeq.n_frame = 0;
  if ( format == NULL ) {
    return retSeq;
  }
  calAlpha = alpha;
  frames = abs(fade_inout);
  frames = MY_MAX(frames,n_file);
  setAlpha = (fade_inout<=0)?alpha:0;

  retSeq.model = mqoCreateList(n_file);
  for ( seq = 0; seq < frames; seq++ ) {
    if ( seq < n_file ) {
      sprintf(filename,format,seq);
    }
    if ( (fade_inout !=  0) && ((frames-1) == seq) ) {
      setAlpha = (fade_inout<0)?0:calAlpha;
    }
    iret = mqoCreateListObject(retSeq.model,seq,filename,scale,(unsigned char)setAlpha);
    if ( iret == - 1 ) {
      seq--;
      mqoClearObject(retSeq.model,seq,n_file-seq);
      break;
    }
    if ( fade_inout !=  0 ) {
      if ( fade_inout<0 ) {
	if ( (n_file-seq) <= (-1*fade_inout) ) {
	  setAlpha -= (calAlpha/(-1*fade_inout));
	  if ( setAlpha < 0 ) setAlpha = 0;
	}
      }
      else {
	setAlpha += (calAlpha/fade_inout);
	if ( calAlpha < setAlpha ) setAlpha = calAlpha;
      }
    }
  }
  retSeq.n_frame = seq;
  return retSeq;
}


/*=========================================================================
  �ڴؿ���mqoCreateSequence
  �����ӡ�Ϣ�֤�MQO�ե����뤫��MQO�������󥹤��������
  �ڰ�����
  format		�ե�����̾�Υե����ޥå�
  n_file		�ե������
  scale		����Ψ��1.0�Ǥ��Τޤޡ�

  �����͡�MQO_SEQUENCE��MQO�������󥹡�
  �����͡�Ϣ�֤�0���鳫��
  =========================================================================*/

MQO_SEQUENCE mqoCreateSequence(const char *format, int n_file, double scale)
{
  return mqoCreateSequenceEx(format, n_file, scale, 0, (unsigned char)255);
}


/*=========================================================================
  �ڴؿ���mqoCallModel
  �����ӡ�MQO��ǥ��OpenGL�β��̾�˸ƤӽФ�
  �ڰ�����
  model		MQO��ǥ�

  �����ۤ͡ʤ�
  =========================================================================*/

void mqoCallModel(MQO_MODEL model)
{
  mqoCallListObject(model, 0);
}


/*=========================================================================
  �ڴؿ���mqoCallSequence
  �����ӡ�MQO�������󥹤�OpenGL�β��̤˸ƤӽФ�
  �ڰ�����
  seq		MQO��������
  i		�ե졼���ֹ�

  �����ۤ͡ʤ�
  �ڻ��͡�MQO�������󥹤��椫����ꤷ���ե졼���ֹ�Υ�ǥ��ƤӽФ�
  =========================================================================*/

void mqoCallSequence(MQO_SEQUENCE seq, int i)
{
  if ( i>=0 && i<seq.n_frame ) {
    mqoCallListObject(seq.model,i);
  }
}


/*=========================================================================
  �ڴؿ���mqoClearObject
  �����ӡ�MQO���֥������ȤΥ��ꥢ
  �ڰ�����
  object	MQO���֥�����������
  from	��������ֹ��0����
  num		�������Ŀ�

  �����ۤ͡ʤ�
  =========================================================================*/

void mqoClearObject( MQO_OBJECT object[], int from, int num ) 
{
  int					loop, o, m;
  MQO_INNER_OBJECT	*obj;
  MQO_MATERIAL		*mat;

  if ( object == NULL ) return;

  for ( loop = from; loop < from + num; loop++ ) {
    for ( o = 0; o < (object+from)->objnum; o++ ) {

      obj = &(object+loop)->obj[o];
      for ( m = 0; m < obj->matnum; m++ ) {
	mat = &obj->mat[m];
	if ( mat->datanum <= 0 ) continue;
	if ( g_isVBOSupported ) {
	  // ĺ���Хåե��κ��
#if 0	  
	  glDeleteBuffersARB( 1, &mat->VBO_id );
#endif	  
	}

	// ĺ������κ��
	if ( mat->isUseTexture ) {
	  if ( mat->vertex_t != NULL ) {
	    free(mat->vertex_t);
	    mat->vertex_t = NULL;
	  }
	}
	else {
	  if ( mat->vertex_p != NULL ) {
	    free(mat->vertex_p);
	    mat->vertex_p = NULL;
	  }
	}
      }
      if ( obj->mat != NULL ) {
	free(obj->mat);
	obj->mat = NULL;
      }
      obj->matnum = 0;
    }
  }

}


/*=========================================================================
  �ڴؿ���mqoDeleteObject
  �����ӡ�MQO���֥������Ȥ�������
  �ڰ�����
  object	MQO���֥�����������
  num		�������Ŀ�

  �����ۤ͡ʤ�
  =========================================================================*/

void mqoDeleteObject(MQO_OBJECT object[], int num)
{
  mqoClearObject(object,0,num);
  free(object);
}


/*=========================================================================
  �ڴؿ���mqoDeleteModel
  �����ӡ�MQO��ǥ��������
  �ڰ�����
  model	MQO��ǥ�

  �����ۤ͡ʤ�
  �����͡ۺ��������Ԥä��ѿ�������Ѥ����ǽ�����������
  ���δؿ��μ¹Ը��NULL���������Ƥ�������
  =========================================================================*/

void mqoDeleteModel(MQO_MODEL model)
{
  mqoDeleteObject(model,1);
}


/*=========================================================================
  �ڴؿ���mqoDeleteSequence
  �����ӡ�MQO�������󥹤�������
  �ڰ�����
  seq		MQO��������

  �����͡ۺ��������Ԥä��ѿ�������Ѥ����ǽ�����������
  ���δؿ��μ¹Ը��NULL���������Ƥ�������
  =========================================================================*/

void mqoDeleteSequence(MQO_SEQUENCE seq)
{
  mqoDeleteObject( seq.model, seq.n_frame );
}
