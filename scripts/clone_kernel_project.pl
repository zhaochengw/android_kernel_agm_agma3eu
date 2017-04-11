#!/usr/bin/perl -w
#
# Usage:
#   perl clone_kernel_project.pl origin new
#

use strict;
use File::Find;

my $origin = shift @ARGV;
my $new = shift @ARGV;

my @find_files;
my $find_dir;
my $build_script;
my $dts_path = "arch/arm/boot/dts/qcom/his";

sub find_defconfig() {
	if (/^${origin}_defconfig/ or /^${origin}_release_defconfig/) {
		$find_dir = $File::Find::dir;
		push(@find_files, $_);
	}
}

sub modify_file() {
	my ($infile) = @_;

	open IN, "< $infile" or die "open $infile failed\n";
	open OUT, "> temp.file" or die "open temp.file failed\n";
	while (<IN>) {
		my $line = $_;
		if ($line =~ /$origin/) {
			$line =~ s/$origin/$new/;
		} 

		print OUT $line;
	}
	close OUT;
	close IN;
	system("mv temp.file $infile");
}

# copy defconfig
find(\&find_defconfig, "arch/arm/configs");
find(\&find_defconfig, "arch/arm64/configs");
foreach my $fname (@find_files) {
	my $oldname = $fname;
	$fname =~ s/$origin/$new/;
	system("cp $find_dir/$oldname $find_dir/$fname");
	&modify_file("$find_dir/$fname");
}

# copy device tree
system("cp -rf $dts_path/$origin $dts_path/$new");
system("cp $dts_path/${origin}.dts $dts_path/${new}.dts");
&modify_file("$dts_path/${new}.dts");

# copy build*.sh
$build_script = `ls build_*${origin}.sh`;
chomp($build_script);
system("cp $build_script build_${new}.sh");
&modify_file("./build_${new}.sh");

print "\n\t注意:\n";
print "\n\t  复制完成，请修改编译脚本名称，添加设计代号!\n\n";
