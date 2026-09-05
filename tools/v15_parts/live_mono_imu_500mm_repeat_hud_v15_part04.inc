    VioState avg = samples.back();
    avg.px = avg.py = avg.pz = 0.0;
    avg.vx = avg.vy = avg.vz = 0.0;
    for (const auto& q : samples) {
      avg.px += q.px; avg.py += q.py; avg.pz += q.pz;
      avg.vx += q.vx; avg.vy += q.vy; avg.vz += q.vz;
    }
    const double n = static_cast<double>(samples.size());
    avg.px /= n; avg.py /= n; avg.pz /= n;
    avg.vx /= n; avg.vy /= n; avg.vz /= n;
    out->s = avg;
    out->valid = true;
    reset();
    return true;
  }

  int count() const { return static_cast<int>(samples.size()); }
};

struct V8Leg {
  Leg measured;
  Mark start_press;
  Mark end_press;
  double start_settle_shift_m = 0.0;
  double end_settle_shift_m = 0.0;
};

double markDistance(const Mark& a, const Mark& b) {
  if (!a.valid || !b.valid) return 0.0;
  const double dx = b.s.px-a.s.px;
  const double dy = b.s.py-a.s.py;
  const double dz = b.s.pz-a.s.pz;
  return std::sqrt(dx*dx+dy*dy+dz*dz);
}

Leg makeLeg500(int idx,const Mark&a,const Mark&b){
  Leg r;
  r.index=idx;
  r.direction=(idx%2==0)?"A->B":"B->A";
  r.a=a; r.b=b;
  r.dx=b.s.px-a.s.px;
  r.dy=b.s.py-a.s.py;
  r.dz=b.s.pz-a.s.pz;
  r.horizontal=std::sqrt(r.dx*r.dx+r.dy*r.dy);
  r.d3=std::sqrt(r.horizontal*r.horizontal+r.dz*r.dz);
  r.error=r.horizontal-kTruth500M;
  r.scale=r.horizontal/kTruth500M;
  return r;
}

bool v8MeasurementPass(const std::vector<V8Leg>& rows, double* sd_out=nullptr) {
  if (rows.size() != kLegCount) return false;
  double sum=0.0, sum2=0.0;
  for (const auto& row : rows) {
    const Leg& r = row.measured;
    const double error500 = r.horizontal - kTruth500M;
    if (std::abs(error500) > kV8MaxLegErrorM || std::abs(r.dz) > kV8MaxAbsDzM) return false;
    sum += r.horizontal;
    sum2 += r.horizontal*r.horizontal;
  }
  const double mean = sum / rows.size();
  const double sd = std::sqrt(std::max(0.0, sum2/rows.size()-mean*mean));
  if (sd_out) *sd_out = sd;
  if (sd > kV8MaxStdM) return false;
  for (size_t i=0; i+1<rows.size(); i+=2) {
    if (pairAngleDeg(rows[i].measured, rows[i+1].measured) < kV8MinPairAngleDeg) return false;
  }
  return true;
}

void saveCsvV8(const std::vector<V8Leg>& rows) {
  std::ofstream f(kV8CsvPath, std::ios::trunc);
  if (!f) return;
  f << "leg,direction,start_press_kf,start_settled_kf,end_press_kf,end_settled_kf,"
       "start_press_px,start_press_py,start_press_pz,start_px,start_py,start_pz,"
       "end_press_px,end_press_py,end_press_pz,end_px,end_py,end_pz,"
       "start_settle_shift_m,end_settle_shift_m,dx_m,dy_m,dz_m,horizontal_m,"
       "distance3d_m,error_horizontal_m,scale_horizontal,pair_angle_deg\n";
  f << std::fixed << std::setprecision(9);
  for (size_t i=0;i<rows.size();++i) {
    const auto& row=rows[i];
    const auto& r=row.measured;
