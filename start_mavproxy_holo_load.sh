set -e

sudo mavproxy.py --master tcp:127.0.0.1:5760 --cmd "module load hologram;" --out=udp:127.0.0.1:14550
