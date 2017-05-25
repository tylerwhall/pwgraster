extern crate byteorder;
extern crate png;
extern crate pwgraster;

use std::io::Cursor;
use std::io::{BufReader, Read, Write};
use std::path::Path;

use png::HasParameters;
use pwgraster::{PWG_HEADER_SIZE, PWG_SYNC_WORD, PageHeader};

fn main() {
    let f = std::env::args()
        .nth(1)
        .expect("Specify input file as first argument");
    let outpath = Path::new(&f).with_extension("png");
    let infile = std::fs::File::open(f).expect("Failed to open input");
    let mut infile = BufReader::new(infile);

    let mut sync = [0; 4];
    infile.read_exact(&mut sync[..]).unwrap();
    if &sync != PWG_SYNC_WORD {
        panic!("Bad sync word {:?}", sync);
    }

    let mut buf = [0; PWG_HEADER_SIZE];
    infile.read_exact(&mut buf[..]).unwrap();
    let header = PageHeader::from_buf(&buf).unwrap();
    header.validate().unwrap();

    // Unwrap safe because of validate()
    let width = header.Width().unwrap();
    let height = header.Height().unwrap();
    let colors = header.NumColors().unwrap();
    let bpp = header.BitsPerPixel().unwrap();
    let bpc = header.BitsPerColor().unwrap();
    println!("Dimensions {} x {}", width, height);
    println!("{} colors. {} bpp. {} bpc", colors, bpp, bpc);

    println!("Output file {:?}", outpath);
    let outfile = std::fs::File::create(outpath).unwrap();
    let mut encoder = png::Encoder::new(outfile, width, height);

    match colors {
        1 => encoder.set(png::ColorType::Grayscale),
        3 => encoder.set(png::ColorType::RGB),
        _ => panic!("Unsupported # of colors {}", colors),
    };

    match bpc {
        8 => encoder.set(png::BitDepth::Eight),
        16 => encoder.set(png::BitDepth::Sixteen),
        _ => panic!("Unsupported depth"),
    };

    let mut writer = encoder.write_header().unwrap();

    let outbuf = Vec::with_capacity(header.image_size_bytes().unwrap());
    let mut outcursor = Cursor::new(outbuf);

    header.unpack_page(&mut infile, &mut outcursor).unwrap();
    writer.write_image_data(&outcursor.into_inner()).unwrap();
}
