
use strict;

my $start = 0;
my $end   = 0;
my $project;
my @c_files;

sub is_correct_date_format()
{
	my ($date_str) = @_;

	return 1 if ($date_str =~ /\d{4}-\d{2}-\d{2}/);
	return 0;
}

sub is_correct_commit_id()
{
	my ($commit_id) = @_;

	return 1 if ($commit_id =~ /\w{40}/);
	return 0;
}

sub get_modified_sourcefile_list()
{
	my ($since, $until) = @_;

	system("git log $since..HEAD --stat > gitlog.log");
	open LOG, "< gitlog.log" or die "open gitlog.log file failed!";
	while(<LOG>)
	{
		if (/\s+(.*\.c)\s+/) {
			push(@c_files, $1);
		}
	}
	close LOG;
	system("rm -rf gitlog.log");
}

if ($#ARGV < 1) {
	print "\n\tPlease input start date, format: yyyy-mm-dd\n\n";
	exit 0;
}

$project = shift @ARGV;
$start = shift @ARGV;
$end = shift @ARGV;

# input parameter check
if (not -e "build_$project.sh") {
	print "\n\tInput project is not exsit\n\n";
	exit(0);
}

if (!&is_correct_commit_id($start)) {
	print "\n\tThe start commit id is error\n\n";
	exit 0;
}

&get_modified_sourcefile_list($start, $end);

system("rm -f warns.txt");
system("echo -e \"check commit: $start .. $end\n\n\" > warns.txt");
foreach my $file (@c_files) {
	system(". ./build_$project.sh check --output-file $file");
}

