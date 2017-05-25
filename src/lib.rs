extern crate byteorder;
#[macro_use]
extern crate enum_primitive;
extern crate memchr;

use std::io::{self, Read, Write};
use byteorder::ReadBytesExt;

#[derive(Debug, PartialEq, Eq)]
pub enum ErrorKind {
    UnterminatedString,
    InvalidUTF8,
    BadPwgRasterString,
    BadEnumValue(u32),
    /// A reserved field contained a non-zero byte
    NonZeroReserved,
    /// The unused high byte was set in SrgbColor
    InvalidColor,
}

#[derive(Debug)]
pub struct Error {
    field: &'static str,
    offset: usize,
    err: ErrorKind,
}

#[derive(Debug)]
pub enum UnpackError {
    HeaderError(Error),
    IO(io::Error),
    /// Pixel repeat exceeded line length
    LineOverflow,
    /// Line repeat exceeded image height
    HeightOverflow,
    /// bpp is not 1 or a multiple of 8
    UnsupportedBpp,
}

impl From<Error> for UnpackError {
    fn from(e: Error) -> Self {
        UnpackError::HeaderError(e)
    }
}

impl From<io::Error> for UnpackError {
    fn from(e: io::Error) -> Self {
        UnpackError::IO(e)
    }
}

pub mod types {
    use std::str;
    use byteorder::{ByteOrder, BigEndian};
    use enum_primitive::FromPrimitive;
    use memchr::memchr;
    use ErrorKind;

    pub trait PWGType<'a>: Sized {
        fn deserialize(buf: &'a [u8]) -> Result<Self, ErrorKind>;
    }

    // Marker for enums that should be represented as u32.
    // This is currently all of them.
    pub trait EnumU32: FromPrimitive {}

    impl<'a, T: EnumU32> PWGType<'a> for T {
        fn deserialize(buf: &'a [u8]) -> Result<Self, ErrorKind> {
            let val = BigEndian::read_u32(buf);
            Self::from_u32(val).ok_or(ErrorKind::BadEnumValue(val))
        }
    }

    // 4.3.1.1
    pub type Boolean = bool;

    impl<'a> PWGType<'a> for Boolean {
        fn deserialize(buf: &'a [u8]) -> Result<Self, ErrorKind> {
            let val = BigEndian::read_u32(buf);
            Ok(val != 0)
        }
    }

    // 4.3.1.2
    #[derive(Debug, PartialEq, Eq)]
    pub struct CString<'a>(pub &'a str);

    impl<'a> PWGType<'a> for CString<'a> {
        fn deserialize(buf: &'a [u8]) -> Result<Self, ErrorKind> {
            if let Some(end) = memchr(0, buf) {
                str::from_utf8(&buf[0..end])
                    .map(|s| CString(s))
                    .map_err(|_| ErrorKind::InvalidUTF8)
            } else {
                Err(ErrorKind::UnterminatedString)
            }
        }
    }

    #[test]
    fn test_cstring() {
        assert_eq!(CString::deserialize(b"abcd\0abcd").unwrap().0, "abcd")
    }

    // 4.3.1.3
    enum_from_primitive! {
    pub enum ColorOrder {
        // Chunky pixels, e.g. CMYK CMYK CMYK ...
        Chunky = 0,
    }
    }

    impl EnumU32 for ColorOrder {}

    // 4.3.1.4
    enum_from_primitive! {
    pub enum ColorSpace {
        // Device RGB (red green blue)
        Rgb = 1,
        // Device black
        Black = 3,
        // Device CMYK (cyan magenta yellow black)
        Cmyk = 6,
        // sRGB grayscale
        Sgray = 18,
        // sRGB color
        Srgb = 19,
        // Adobe RGB color
        AdobeRgb = 20,
        // Device color, 1 colorant
        Device1 = 48,
        // Device color, 2 colorants
        Device2 = 49,
        // Device color, 3 colorants
        Device3 = 50,
        // Device color, 4 colorants
        Device4 = 51,
        // Device color, 5 colorants
        Device5 = 52,
        // Device color, 6 colorants
        Device6 = 53,
        // Device color, 7 colorants
        Device7 = 54,
        // Device color, 8 colorants
        Device8 = 55,
        // Device color, 9 colorants
        Device9 = 56,
        // Device color, 10 colorants
        Device10 = 57,
        // Device color, 11 colorants
        Device11 = 58,
        // Device color, 12 colorants
        Device12 = 59,
        // Device color, 13 colorants
        Device13 = 60,
        // Device color, 14 colorants
        Device14 = 61,
        // Device color, 15 colorants
        Device15 = 62,
    }
    }

    impl EnumU32 for ColorSpace {}

    // 4.3.1.5
    enum_from_primitive! {
    pub enum Edge {
        // The short edge of the media is first.
        ShortEdgeFirst = 0,
        // The long edge of the media is first.
        LongEdgeFirst = 1,
    }
    }

    impl EnumU32 for Edge {}

    // 4.3.1.6
    pub type Integer = i32;

    impl<'a> PWGType<'a> for Integer {
        fn deserialize(buf: &'a [u8]) -> Result<Self, ErrorKind> {
            Ok(BigEndian::read_i32(buf))
        }
    }

    // 4.3.1.7
    enum_from_primitive! {
    pub enum MediaPosition {
        // Default or automatically selected source.
        Auto = 0,
        // The primary or main source.
        Main = 1,
        // The secondary or alternate source.
        Alternate = 2,
        // The large capacity source.
        LargeCapacity = 3,
        // The manual feed source.
        Manual = 4,
        // The envelope feed source.
        Envelope = 5,
        // The CD/DVD/Bluray disc source.
        Disc = 6,
        // The photo media source.
        Photo = 7,
        // The Hagaki media source.
        Hagaki = 8,
        // The primary or main roll.
        MainRoll = 9,
        // The secondary or alternate roll.
        AlternateRoll = 10,
        // The topmost source.
        Top = 11,
        // The middle source.
        Middle = 12,
        // The bottommost source.
        Bottom = 13,
        // The side source.
        Side = 14,
        // The leftmost source.
        Left = 15,
        // The rightmost source.
        Right = 16,
        // The center source.
        Center = 17,
        // The rear source.
        Rear = 18,
        // The by-pass or multi-purpose source.
        ByPassTray = 19,
        // Tray 1.
        Tray1 = 20,
        // Tray 2.
        Tray2 = 21,
        // Tray 3.
        Tray3 = 22,
        // Tray 4.
        Tray4 = 23,
        // Tray 5.
        Tray5 = 24,
        // Tray 6.
        Tray6 = 25,
        // Tray 7.
        Tray7 = 26,
        // Tray 8.
        Tray8 = 27,
        // Tray 9.
        Tray9 = 28,
        // Tray 10.
        Tray10 = 29,
        // Tray 11.
        Tray11 = 30,
        // Tray 12.
        Tray12 = 31,
        // Tray 13.
        Tray13 = 32,
        // Tray 14.
        Tray14 = 33,
        // Tray 15.
        Tray15 = 34,
        // Tray 16.
        Tray16 = 35,
        // Tray 17.
        Tray17 = 36,
        // Tray 18.
        Tray18 = 37,
        // Tray 19.
        Tray19 = 38,
        // Tray 20.
        Tray20 = 39,
        // Roll 1.
        Roll1 = 40,
        // Roll 2.
        Roll2 = 41,
        // Roll 3.
        Roll3 = 42,
        // Roll 4.
        Roll4 = 43,
        // Roll 5.
        Roll5 = 44,
        // Roll 6.
        Roll6 = 45,
        // Roll 7.
        Roll7 = 46,
        // Roll 8.
        Roll8 = 47,
        // Roll 9.
        Roll9 = 48,
        // Roll 10.
        Roll10 = 49,
    }
    }

    impl EnumU32 for MediaPosition {}

    // 4.3.1.8
    enum_from_primitive! {
    pub enum Orientation {
        //Not rotated
        Portrait = 0,
        // Rotated 90 degrees counter-clockwise
        Landscape = 1,
        // Rotated 180 degrees
        ReversePortrait = 2,
        // Rotated 90 degrees clockwise
        ReverseLandscape = 3,
    }
    }

    impl EnumU32 for Orientation {}

    // 4.3.1.9
    enum_from_primitive! {
    pub enum PrintQuality {
        // The default print quality.
        Default = 0,
        // Draft/fast print quality.
        Draft = 3,
        // Normal print quality.
        Normal = 4,
        // High/best/photo print quality.
        High = 5,
    }
    }

    impl EnumU32 for PrintQuality {}

    // 4.3.1.10
    pub struct Reserved;

    impl<'a> PWGType<'a> for Reserved {
        fn deserialize(buf: &'a [u8]) -> Result<Self, ErrorKind> {
            for byte in buf {
                if *byte != 0 {
                    return Err(ErrorKind::NonZeroReserved);
                }
            }
            Ok(Reserved)
        }
    }

    // 4.3.1.11
    pub struct SrgbColor(u32);

    impl SrgbColor {
        pub fn red(&self) -> u8 {
            (self.0 >> 16) as u8
        }

        pub fn green(&self) -> u8 {
            (self.0 >> 8) as u8
        }

        pub fn blue(&self) -> u8 {
            self.0 as u8
        }
    }

    impl<'a> PWGType<'a> for SrgbColor {
        fn deserialize(buf: &'a [u8]) -> Result<Self, ErrorKind> {
            let val = BigEndian::read_u32(buf);
            if (val & 0xff000000) != 0 {
                Err(ErrorKind::InvalidColor)
            } else {
                Ok(SrgbColor(val))
            }
        }
    }

    // 4.3.1.12
    pub type UnsignedInteger = u32;

    impl<'a> PWGType<'a> for UnsignedInteger {
        fn deserialize(buf: &'a [u8]) -> Result<Self, ErrorKind> {
            Ok(BigEndian::read_u32(buf))
        }
    }

    // 4.3.1.13
    pub struct VendorData<'a>(&'a [u8]);

    impl<'a> PWGType<'a> for VendorData<'a> {
        fn deserialize(buf: &'a [u8]) -> Result<Self, ErrorKind> {
            Ok(VendorData(buf))
        }
    }

    // 4.3.1.14
    enum_from_primitive! {
    pub enum When {
        // Never apply feature.
        Never = 0,
        // Apply feature after current document/file.
        AfterDocument = 1,
        // Apply feature after current job.
        AfterJob = 2,
        // Apply feature after current set/copy.
        AfterSet = 3,
        // Apply feature after current page.
        AfterPage = 4,
    }
    }

    impl EnumU32 for When {}

    // 4.3.2.1
    pub struct PwgRaster<'a>(pub CString<'a>);

    impl<'a> PWGType<'a> for PwgRaster<'a> {
        fn deserialize(buf: &'a [u8]) -> Result<Self, ErrorKind> {
            let s = CString::deserialize(buf)?;
            if s.0 == "PwgRaster" {
                Ok(PwgRaster(s))
            } else {
                Err(ErrorKind::BadPwgRasterString)
            }
        }
    }

    // 4.3.2.2
    pub struct HwResolution {
        pub feed_res_dpi: UnsignedInteger,
        pub cross_feed_res_dpi: UnsignedInteger,
    }

    impl<'a> PWGType<'a> for HwResolution {
        fn deserialize(buf: &'a [u8]) -> Result<Self, ErrorKind> {
            Ok(HwResolution {
                   feed_res_dpi: UnsignedInteger::deserialize(&buf[..4])?,
                   cross_feed_res_dpi: UnsignedInteger::deserialize(&buf[4..])?,
               })
        }
    }

    // 4.3.3.11
    pub struct PageSize {
        pub width: UnsignedInteger,
        pub height: UnsignedInteger,
    }

    impl<'a> PWGType<'a> for PageSize {
        fn deserialize(buf: &'a [u8]) -> Result<Self, ErrorKind> {
            Ok(PageSize {
                   width: UnsignedInteger::deserialize(&buf[..4])?,
                   height: UnsignedInteger::deserialize(&buf[4..])?,
               })
        }
    }
}

use types::*;

macro_rules! make_struct {
    ( pub struct $name:ident {
            $(
                $start:expr, $end:expr, $fname:ident: $ftype:ident
            ),*
    } ) => {
        pub struct $name<'a>(&'a [u8]);
        impl<'a> $name<'a> {
                #[inline]
                // Do the full length check to help optimize out subsequent checks
                // for fields within the struct
                fn checked_buf(&self) -> &[u8] {
                    &self.0[..PWG_HEADER_SIZE]
                }
            $(
                #[allow(non_snake_case)]
                #[inline]
                pub fn $fname(&self) -> Result<$ftype, Error> {
                    <$ftype as PWGType>::deserialize(&self.checked_buf()[$start..$end + 1])
                        .map_err(|e| {
                            Error {
                                field: stringify!($fname),
                                offset: $start,
                                err: e,
                            }
                        })
                }
             )*
                /// Attempt to unpack all values, calling a function to
                /// handle any errors as they are encountered.
                ///
                /// An error handler may choose to map the errors to a custom
                /// type or discard some types of errors.
                pub fn validate_filter<E, F: Fn(Error) -> Result<(), E>>(&self, err_handler: F) -> Result<(), E> {
                    $(
                        if let Err(e) = self.$fname() {
                            if let Err(e) = err_handler(e) {
                                return Err(e);
                            }
                        }
                     )*
                    Ok(())
                }
        }
    }
}

pub const PWG_SYNC_WORD: &'static [u8; 4] = b"RaS2";
pub const PWG_HEADER_SIZE: usize = 1796;

make_struct! {
    pub struct PageHeader {
        0, 63, PwgRaster: CString,
        64, 127, MediaColor: CString,
        128, 191, MediaType: CString,
        192, 255, PrintContentOptimize: CString,
        256, 267, Reserved1: Reserved,
        268, 271, CutMedia: When,
        272, 275, Duplex: Boolean,
        276, 283, HWResolution: HwResolution,
        284, 299, Reserved2: Reserved,
        300, 303, InsertSheet: Boolean,
        304, 307, Jog: When,
        308, 311, LeadingEdge: Edge,
        312, 323, Reserved3: Reserved,
        324, 327, MediaPosition: MediaPosition,
        328, 331, MediaWeight: UnsignedInteger,
        332, 339, Reserved4: Reserved,
        340, 343, NumCopies: UnsignedInteger,
        344, 347, Orientation: Orientation,
        348, 351, Reserved5: Reserved,
        352, 359, PageSize: PageSize,
        360, 367, Reserved6: Reserved,
        368, 371, Tumble: Boolean,
        372, 375, Width: UnsignedInteger,
        376, 379, Height: UnsignedInteger,
        380, 383, Reserved7: Reserved,
        384, 387, BitsPerColor: UnsignedInteger,
        388, 391, BitsPerPixel: UnsignedInteger,
        392, 395, BytesPerLine: UnsignedInteger,
        396, 399, ColorOrder: ColorOrder,
        400, 403, ColorSpace: ColorSpace,
        404, 419, Reserved8: Reserved,
        420, 423, NumColors: UnsignedInteger,
        424, 451, Reserved9: Reserved,
        452, 455, TotalPageCount: UnsignedInteger,
        456, 459, CrossFeedTransform: Integer,
        460, 463, FeedTransform: Integer,
        464, 467, ImageBoxLeft: UnsignedInteger,
        468, 471, ImageBoxTop: UnsignedInteger,
        472, 475, ImageBoxRight: UnsignedInteger,
        476, 479, ImageBoxBottom: UnsignedInteger,
        480, 483, AlternatePrimary: SrgbColor,
        484, 487, PrintQuality: PrintQuality,
        488, 507, Reserved10: Reserved,
        508, 511, VendorIdentifier: UnsignedInteger,
        512, 515, VendorLength: UnsignedInteger,
        516, 1603, VendorData: VendorData,
        1604, 1667, Reserved11: Reserved,
        1668, 1731, RenderingIntent: CString,
        1732, 1795, PageSizeName: CString
    }
}

impl<'a> PageHeader<'a> {
    /// Construct over a borrowed buffer that starts at the beginning of the
    /// PWG header (after the sync word or a previous page).
    ///
    /// Does not copy the input or allocate.
    ///
    /// Returns None if the buffer is not at least PWG_HEADER_SIZE bytes. No
    /// input validation is performed at this step.
    #[inline]
    pub fn from_buf(buf: &'a [u8]) -> Option<Self> {
        if buf.len() >= PWG_HEADER_SIZE {
            Some(PageHeader(buf))
        } else {
            None
        }
    }

    /// Attempt to unpack all values, calling a function to
    /// handle any errors as they are encountered.
    ///
    /// This ignores reserved fields but returns the first error encountered in
    /// other fields.
    pub fn validate(&self) -> Result<(), Error> {
        self.validate_filter(|e| match e {
                                 Error { err: ErrorKind::NonZeroReserved, .. } => Ok(()),
                                 _ => Err(e),
                             })
    }

    /// Attempt to unpack all values, calling a function to
    /// handle any errors as they are encountered.
    ///
    /// Warning! This validates that reserved fields are set to 0 so code using
    /// this may not be future-proof.
    pub fn validate_withreserved(&self) -> Result<(), Error> {
        self.validate_filter(|e| Err(e))
    }

    /// Return the image's smallest unit of pixel values in bytes. This is the
    /// bpp of the image or 1 in the case of packed 1-bit pixels.
    fn chunk_bytes(&self) -> Result<usize, UnpackError> {
        match self.BitsPerPixel()? {
            // 1bpp is packed into bytes
            1 => Ok(1),
            // Else divide bpp by 8
            bpp if bpp % 8 == 0 => Ok(bpp as usize / 8),
            _ => Err(UnpackError::UnsupportedBpp),
        }
    }

    /// The size of an unpacked line.
    ///
    /// 1 bpp data will remain packed 8 pixel in a byte. Pixels are packed into
    /// a stream of bytes, i.e. 2 24bpp pixels are represented as 6 contiguous
    /// bytes.
    pub fn bytes_per_line(&self) -> Result<usize, Error> {
        Ok((self.Width()? as usize * self.BitsPerPixel()? as usize + 7) / 8)
    }

    /// The size of the unpacked image data in bytes. bytes_per_line * height.
    ///
    /// 1 bpp data will remain packed 8 pixel in a byte. Pixels are packed into
    /// a stream of bytes, i.e. 2 24bpp pixels are represented as 6 contiguous
    /// bytes.
    pub fn image_size_bytes(&self) -> Result<usize, Error> {
        Ok(self.bytes_per_line()? * self.Height()? as usize)
    }

    pub fn unpack_page<R: Read, W: Write>(&self,
                                          reader: &mut R,
                                          writer: &mut W)
                                          -> Result<(), UnpackError> {
        let chunk_bytes = self.chunk_bytes()?;
        let bytes_per_line = self.bytes_per_line()?;
        let height = self.Height()? as usize;

        unpack_page(reader, writer, chunk_bytes, bytes_per_line, height)
    }
}

pub fn unpack_page<R: Read, W: Write>(reader: &mut R,
                                      writer: &mut W,
                                      chunk_bytes: usize,
                                      bytes_per_line: usize,
                                      height: usize)
                                      -> Result<(), UnpackError> {
    let mut pixel_buf = Vec::with_capacity(chunk_bytes);
    pixel_buf.resize(chunk_bytes, 0);
    let mut line_buf = Vec::with_capacity(bytes_per_line);
    line_buf.resize(bytes_per_line, 0);

    let mut lines_complete = 0;
    while lines_complete < height {
        let line_repeat = reader.read_u8()? as usize + 1;
        let mut line_bytes = 0;

        if (lines_complete + line_repeat) > height {
            return Err(UnpackError::HeightOverflow);
        }

        while line_bytes < bytes_per_line {
            let count = reader.read_u8()?;
            if count < 128 {
                let repeat_count = count + 1;
                reader.read_exact(&mut pixel_buf[..])?;

                let repeat_length = pixel_buf.len() * repeat_count as usize;
                if (bytes_per_line - line_bytes) < repeat_length {
                    return Err(UnpackError::LineOverflow);
                }

                for _ in 0..repeat_count {
                    line_buf[line_bytes..line_bytes + pixel_buf.len()]
                        .clone_from_slice(&pixel_buf[..]);
                    line_bytes += pixel_buf.len();
                }
            } else {
                let count = 257 - count as usize;
                let byte_count = count * pixel_buf.len();

                if (bytes_per_line - line_bytes) < byte_count {
                    return Err(UnpackError::LineOverflow);
                }
                reader
                    .read_exact(&mut line_buf[line_bytes..line_bytes + byte_count])?;
                line_bytes += byte_count;
            }
        }

        for _ in 0..line_repeat {
            writer.write_all(&line_buf[..])?;
            lines_complete += 1;
        }
    }
    Ok(())
}

#[test]
fn test_unpack() {
    use std::io::Cursor;
    let mut buf = [0, 0xfd, 1, 2, 3, 0];
    let mut outbuf = [0; 4];

    unpack_page(&mut Cursor::new(&buf[..]),
                &mut Cursor::new(&mut outbuf[..]),
                1,
                4,
                1)
            .unwrap();
    assert_eq!(outbuf, [1, 2, 3, 0]);
    let e = unpack_page(&mut Cursor::new(&buf[..]),
                        &mut Cursor::new(&mut outbuf[..3]),
                        1,
                        4,
                        1);
    if let Err(UnpackError::IO(e)) = e {
        assert_eq!(e.kind(), io::ErrorKind::WriteZero);
    } else {
        panic!("{:?}", e);
    }
    let e = unpack_page(&mut Cursor::new(&buf[..4]),
                        &mut Cursor::new(&mut outbuf[..]),
                        1,
                        4,
                        1);
    if let Err(UnpackError::IO(e)) = e {
        assert_eq!(e.kind(), io::ErrorKind::UnexpectedEof);
    } else {
        panic!("{:?}", e);
    }

    match unpack_page(&mut Cursor::new(&buf[..]),
                      &mut Cursor::new(&mut outbuf[..]),
                      1,
                      3,
                      1) {
        Err(UnpackError::LineOverflow) => (),
        e => panic!("{:?}", e),
    }

    // Set line repeat
    buf[0] = 1;
    match unpack_page(&mut Cursor::new(&buf[..]),
                      &mut Cursor::new(&mut outbuf[..]),
                      1,
                      4,
                      1) {
        Err(UnpackError::HeightOverflow) => (),
        e => panic!("{:?}", e),
    }
}
