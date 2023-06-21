#include <stdio.h>
#include <assert.h>
#include <time.h>
#include <sys/timeb.h>
#include <errno.h>
#include <stdlib.h>
#include <stdint.h>

#import <AppKit/AppKit.h>
#include <OpenGL/gl.h>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/mat2x2.hpp>
#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/matrix_transform.hpp>

_Pragma("GCC diagnostic ignored \"-Wdeprecated-declarations\"");

template<typename T> int print(T arg);
template<typename T,typename ...TV>
int print(T arg,TV ...vargs){
    int x=print<T>(arg);
    return x+print(vargs...);
}
#define PRINT_BASIC_RAW(type,format) \
    template<> int print<type>(type arg){return printf(format,arg);}
#define PRINT_BASIC(type,format) \
    PRINT_BASIC_RAW(type,format) \
    PRINT_BASIC_RAW(type*,"%p") \
    PRINT_BASIC_RAW(const type*,"%p")

PRINT_BASIC_RAW(char,"%c");
PRINT_BASIC_RAW(char*,"%s");
PRINT_BASIC_RAW(void*,"%p");
PRINT_BASIC_RAW(const char*,"%s");
PRINT_BASIC_RAW(const void*,"%p");
PRINT_BASIC(bool,"%d");
PRINT_BASIC(short,"%d");
PRINT_BASIC(int,"%d");
PRINT_BASIC(long,"%ld");
PRINT_BASIC(long long,"%lld");
PRINT_BASIC(unsigned char,"%u");
PRINT_BASIC(unsigned short,"%u");
PRINT_BASIC(unsigned int,"%u");
PRINT_BASIC(unsigned long,"%lu");
PRINT_BASIC(unsigned long long,"%llu");
PRINT_BASIC(float,"%f");
PRINT_BASIC(double,"%f");
PRINT_BASIC(long double,"%Lf");

#undef PRINT_BASIC
#undef PRINT_BASIC_RAW
int println(){return print('\n');}
template<typename ...TV>
int println(TV ...vargs){return print(vargs...,'\n');}
#define lprintln(args...) println(#args,":\t",args)


typedef glm::vec2 float2;
typedef glm::vec3 float3;
typedef glm::vec4 float4;
typedef glm::ivec2 int2;
typedef glm::ivec3 int3;
typedef glm::ivec4 int4;
typedef glm::mat2 float2x2;
typedef glm::mat3 float3x3;
typedef glm::mat4 float4x4;

template<> int print<float2>(float2 v){return print('[',v.x,',',v.y,']');}
template<> int print<float3>(float3 v){return print('[',v.x,',',v.y,',',v.z,']');}
template<> int print<float4>(float4 v){return print('[',v.x,',',v.y,',',v.z,',',v.w,']');}

template<> int print<int2>(int2 v){return print('[',v.x,',',v.y,']');}
template<> int print<int3>(int3 v){return print('[',v.x,',',v.y,',',v.z,']');}
template<> int print<int4>(int4 v){return print('[',v.x,',',v.y,',',v.z,',',v.w,']');}


template<> int print<float3x3>(float3x3 m){
    float3 *cols=(float3*)&m;
    int x=0;
    x+=println('[',cols[0],',');
    x+=println(' ',cols[1],',');
    x+=println(' ',cols[2],']');
    return x;
}

float4x4 scale(float3 v){
    return float4x4(
    v.x,0.,0.,0.,
    0.,v.y,0.,0.,
    0.,0.,v.z,0.,
    0.,0.,0.,1.
    );
}
float4x4 scale(float x,float y,float z){
    return scale(float3(x,y,z));
}


double itime(){
    struct timeb now;
    ftime(&now);
    return (double)(now.time%(60*60*24))+now.millitm/1e3;
}

void print_fps(){
    static double timer;
    double delta=itime()-timer;
    timer+=delta;
    printf("\rfps = %f ",1/delta);
    fflush(stdout);
}

float map(float t,float t0,float t1,float s0,float s1){
    return s0+(s1-s0)*(t-t0)/(t1-t0);
}

void color_wheel(float t,float ret[4]){
    t=fmodf(t,1);
    float theta=map(t,0.,1.,0.,2.*pi);
    float cd=sqrtf(2)/3;
    float3 c={cd,cd,cd};
    float d=sqrtf(2/3.0-2*cd+1);
    float3 i=d*glm::normalize((float3){1,1,-2});
    float3 j=d*glm::normalize(glm::cross(c,i));
    float3 _ret=glm::normalize(c+i*cosf(theta)+j*sinf(theta));
    ret[0]=_ret.x;
    ret[1]=_ret.y;
    ret[2]=_ret.z;
    ret[3]=1;
}

void print_glerror(uint32_t e){
    switch(e){
        #define CASE(val) case val:puts(#val);break
        CASE(GL_NO_ERROR);
        CASE(GL_INVALID_ENUM);
        CASE(GL_INVALID_VALUE);
        CASE(GL_INVALID_OPERATION);
        CASE(GL_STACK_OVERFLOW);
        CASE(GL_STACK_UNDERFLOW);
        CASE(GL_OUT_OF_MEMORY);
        default: puts("unknown glerror");
        #undef CASE
    }
}

constexpr float pos(bool p){return p?1.0f:-1.0f;}

constexpr float deg(float rad){return rad*180/pi;}

struct _quat{
    float s;
    float3 v;
    constexpr _quat():s(1),v(float3(0,0,0)){}
    constexpr _quat(float _s,float3 _v):s(_s),v(_v){}
    constexpr _quat operator*(const _quat q)const{
        return _quat(s*q.s-dot(v,q.v),s*q.v+q.s*v+cross(v,q.v));
    }
    constexpr _quat &operator*=(const _quat q){
        return (*this)=(*this)*q;
    }
    constexpr bool operator==(const _quat q)const{
        auto div=q*conj();
        return abs(div.s)>.99;
    }
    constexpr _quat conj()const{return _quat(s,-v);}
    constexpr float get_angle(){return 2*atan(glm::length(v)/s);}
    constexpr float4x4 to_mtx()const{
        auto i=float3(1,0,0);
        auto j=float3(0,1,0);
        auto k=float3(0,0,1);
        i=(*this*_quat(0,i)*this->conj()).v;
        j=(*this*_quat(0,j)*this->conj()).v;
        k=(*this*_quat(0,k)*this->conj()).v;
        float4x4 _mtx=float4x4(
            i.x,i.y,i.z,0.,
            j.x,j.y,j.z,0.,
            k.x,k.y,k.z,0.,
            0. ,0. ,0. ,1.
        );
        static_assert(sizeof(float4x4)==sizeof(_mtx));
        return _mtx;
    }
};

template<> int print<_quat>(_quat q){return print(q.s,'+',q.v);}

constexpr float3 rotate(float3 v,_quat q){
    return (q*_quat(0,v)*q.conj()).v;
}
_quat angle_axis(float theta,float3 n){
    return _quat(cos(theta/2),sin(theta/2)*glm::normalize(n));
}
_quat from_to(float3 a,float3 b){
    float3 axis=glm::cross(a,b);
    float angle=acos(glm::dot(glm::normalize(a),glm::normalize(b)));
    float mag=glm::length(axis);
    if(mag<1e-6) return _quat(1,float3(0));
    return angle_axis(angle,axis/mag);
}

static constexpr int3 line2cube_idx(uint8_t x){
    x%=27;
    int3 idx;
    idx.x=x/9;
    x=x%9;
    idx.y=x/3;
    idx.z=x%3;
    return idx;
}
static constexpr uint8_t cube2line_idx(int3 i){
    return 9*i.x+3*i.y+i.z;
}

_quat group[24];
uint8_t field_mul_table[24][24];
uint8_t field_rot_point_table[27][24];
uint8_t field_conj_table[24];

__attribute__((constructor(10))) void compute_group(){
    constexpr float sq2inv=0.7071067811865475f;
    group[0]=_quat(1,float3(0));
    group[1]=_quat(sq2inv,float3(sq2inv,0,0));
    group[2]=_quat(sq2inv,float3(0,sq2inv,0));
    group[3]=_quat(sq2inv,float3(0,0,sq2inv));
    
    for(int i=0;i<4;i++){
        auto j=group[i];
        assert(j==j);
        _quat loop=j*j*j*j*j;
        assert(loop==j);
    }
    
    uint64_t old_size=1;
    uint64_t live_size=4;
    for(int l=0;l<2;l++){
        uint64_t size=live_size;
        for(int i=1;i<size;i++){
            for(uint64_t j=old_size;j<size;j++){
                _quat q=group[i]*group[j];
                bool found=false;
                for(int k=0;k<live_size;k++) if(q==group[k]){
                    found=true;
                    break;
                }
                if(!found){
                    assert(live_size<=24);
                    group[live_size]=q;
                    live_size++;
                }
            }
        }
        old_size=size;
    };

//    double t1=itime();
    
//    lprintln(t1-t0);
    
    assert(live_size==24);

    constexpr uint64_t size=24;
    //verify no duplicates
    for(int i=0;i<size;i++){
        for(int j=i+1;j<size;j++){
            if(group[i]==group[j]){
                lprintln(group[i]);
                lprintln(group[j]);
                assert(0);
            }
        }
    }
    //verify the elements form a group
    bool qreached[24]={0};
    for(int i=0;i<24;i++){
        for(int j=0;j<24;j++){
            auto q=group[i]*group[j];
            bool found=false;
            for(int k=0;k<24;k++) if(q==group[k]){
                found=true;
                qreached[k]=true;
                field_mul_table[i][j]=k;
                break;
            }
            if(!found){
                print("error: ",q);
                assert(0);
            }
        }
    }
    for(int i=0;i<size;i++) assert(qreached[i]);
    
    //populate field_rot_point_table
    for(int i=0;i<27;i++){
        float3 p1=float3(line2cube_idx(i))-1.0f;
        for(int j=0;j<24;j++){
            _quat q=group[j];
            float3 p2=glm::round(rotate(p1,q)+1.0f);
            field_rot_point_table[i][j]=cube2line_idx(int3(p2));
        }
    }
    //populate field_conj_table
    for(int i=0;i<24;i++){
        _quat c=group[i].conj();
        bool found=false;
        for(int j=0;j<24;j++) if(group[j]==c){
            found=true;
            field_conj_table[i]=j;
            break;
        }
        assert(found);
    }

    
//    exit(0);
}


struct field_quat{
    uint8_t val:5=0;
    constexpr field_quat operator*(const field_quat q)const{
        return field_quat{field_mul_table[val][q.val]};
    }
    constexpr field_quat &operator*=(const field_quat q){
        return (*this)=(*this)*q;
    }
    constexpr bool operator==(const field_quat q)const{return val==q.val;}
    constexpr field_quat conj()const{return field_quat{field_conj_table[val]};}
};
static_assert(sizeof(field_quat)==1);

_quat field2real_quat(field_quat q){return group[q.val];}
field_quat real2field_quat(_quat q){
    for(uint8_t i=0;i<24;i++) if(group[i]==q) return field_quat{i};
    assert(0);
    return field_quat{0};
}

struct move_t{
    uint8_t ax_en:2;
    uint8_t slice:2;
    uint8_t angle:2;
    move_t get_reverse(){return move_t{ax_en,slice,(uint8_t)((4u-angle)%4)};}
};
static_assert(sizeof(move_t)==1);


struct field_cube{
    union{
        field_quat rots[3][3][3];
        field_quat _rots[27];
    };
    constexpr field_cube(){for(int i=0;i<27;i++) _rots[i].val=0;}
    constexpr field_quat &get_rot(int3 idx){return rots[idx.x][idx.y][idx.z];}
    constexpr field_quat read_rot(int3 idx)const{return rots[idx.x][idx.y][idx.z];}
    constexpr int3 get_idx(int3 idx)const{
        uint8_t i=cube2line_idx(idx);
        uint8_t j=field_rot_point_table[i][read_rot(idx).val];
        return line2cube_idx(j);
    }
    constexpr bool is_solved()const{
        for(int i=0;i<27;i++) if(_rots[i].val!=0) return false;
        return true;
    }
    constexpr bool is_valid()const{
        int3 idxs[27];
        for(int i=0;i<27;i++) idxs[i]=get_idx(line2cube_idx(i));
        for(int i=0;i<27;i++){
            for(int j=i+1;j<27;j++){
                if(idxs[i]==idxs[j]) return false;
            }
        }
        return true;
    }
    constexpr void set_center_to_1(){
        field_quat c=rots[1][1][1].conj();
        field_quat one{0};
        if(c==one) return;
        for(int i=0;i<27;i++){
            _rots[i]*=c;
        }
        assert(rots[1][1][1]==one);
    }
    constexpr bool operator==(const field_cube a)const{
        field_quat c0=rots[1][1][1].conj();
        field_quat c1=a.rots[1][1][1].conj();
        for(int i=0;i<27;i++) if(c0*_rots[i]!=c1*a._rots[i]) return false;
        return true;
    }
    void randomize(uint32_t steps){
        for(int i=0;i<steps;i++){
            uint32_t x=arc4random();
            char axis_en='x'+x%3;
            x/=3;
            uint8_t slice=2*(x%2);
            x/=2;
            int angle=1+(x%3);
            move(axis_en,angle,slice);
        }
    }
    void move(move_t m){move(m.ax_en+'x',m.angle,m.slice);}
    void move(char axis_en,int angle,uint8_t slice){
        if(angle==0) return;
        assert('x'<=axis_en && axis_en<='z');
        int modified=0;
        field_quat nqax{uint8_t(axis_en-'x'+1)};
        if(angle<0) nqax=nqax.conj();
        field_quat nq{0};
        for(int i=0;i<abs(angle);i++) nq*=nqax;
        
        for(int i=0;i<27;i++){
            const int3 idx1=line2cube_idx(i);
            const int3 idx2=get_idx(idx1);
            switch(axis_en){
                case 'x': if(idx2.x!=slice) continue;break;
                case 'y': if(idx2.y!=slice) continue;break;
                case 'z': if(idx2.z!=slice) continue;break;
            }
            field_quat &q=get_rot(idx1);
            q=nq*q;
            modified++;
        }
        assert(modified==9);
    }
};
static_assert(sizeof(field_cube)==27);

bool dumb_search_rec(field_cube &fc,int depth){
    if(depth<=0) return false;
    static const field_cube one;
    if(one==fc) return true;
    for(char axis='x';axis<='z';axis++){
        for(uint8_t slice=0;slice<=2;slice+=2){
            for(int angle=0;angle<=3;angle++){
                field_cube new_fc=fc;
                new_fc.move(axis,angle,slice);
                if(dumb_search_rec(new_fc,depth-1)) return true;
            }
        }
    }
    return false;
}

__attribute__((constructor(11))) void test_field_cube(){
    puts(__func__);
    for(uint8_t i=0;i<24;i++){
        field_quat q{i};
        assert(q*q.conj()==field_quat{0});
    }
    field_cube c;
    assert(c==field_cube());
    c.move('x',1,1);
    assert(c!=field_cube());
    c.move('x',-1,1);
    assert(c==field_cube());
    constexpr uint32_t n=50;
    uint32_t rand_buf[n];
    arc4random_buf(rand_buf,n*sizeof(uint32_t));
    for(uint8_t i=0;i<2*n;i++){
        int j;
        if(i<n) j=i;
        else j=n-(i-n)-1;
        uint32_t x=rand_buf[j];
        char axis_en='x'+x%3;
        x/=3;
        uint8_t slice=x%3;
        x/=3;
        int angle=(x%6)-3;
        if(i>=n) angle*=-1;
        auto old_c=c;
        c.move(axis_en,angle,slice);
//        lprintln(axis_en,' ',angle,' ',slice);
        assert(c.is_valid());
        if(angle) assert(c!=old_c);
        else assert(c==old_c);
    }
    //(RU^{2}D^{-1}BD^{-1})
    assert(c==field_cube());
    
    int i=0;
    do{
        //slice: L:0,R:2,D:0,U:2,F:0,B:2
        c.move('x',1,2);//R
        c.move('y',2,2);//U^2
        c.move('y',1,0);//D^-1
        c.move('z',1,2);//B
        c.move('y',1,0);//D^-1
        i++;
    }while(c!=field_cube{});
    lprintln(i);
    
    {
        auto t0=itime();
        for(int i=0;i<4;i++){
            field_cube fc;
            fc.randomize(i);
            assert(dumb_search_rec(fc,i+1));
        }
        auto t1=itime();
        lprintln(t1-t0);
    }
    
//    static_assert(sizeof(__int128_t)==17);
 
//    exit(0);
}


template<uint32_t n>
void glVertex(glm::vec<n,float,glm::defaultp> v);

template<> void glVertex<2>(float2 v){glVertex2fv((float*)&v);}
template<> void glVertex<3>(float3 v){glVertex3fv((float*)&v);}
template<> void glVertex<4>(float4 v){glVertex4fv((float*)&v);}

struct isec{
    float3 p;
    float3 n;
    float t;
    bool hit;
    int3 idx;
    int3 idx_pos;
    _quat q;
    char not_axis_en;
};

constexpr isec cube_intersect(float3 p,float3 v){
    float d=.5;
    isec s;
    s.hit=false;
    s.t=1e20;
    for(int i=0;i<3;i++){
        for(int j=-1;j<=1;j+=2){
            float3 n(0,0,0);
            n[i]=j;
            float tp=(d-glm::dot(p,n))/glm::dot(v,n);
            float3 ip=p+v*tp;
            if(0<tp && tp<s.t && abs(ip[(i+1)%3])<d && abs(ip[(i+2)%3])<d ){
                s.t=tp;
                s.hit=true;
                s.n=n;
                s.p=ip;
            }
        }
    }
    return s;
}

void draw_test_cube(bool hit,float3 hit_n){
    for(int k=0;k<3;k++){
        float3 rgb{(float)(k==0),(float)(k==1),(float)(k==2)};
        float3 rgb_inv=1.0f-rgb;
        for(int j=0;j<2;j++){
            float3 n(0);
            n[k]=pos(j);
            if(hit && glm::dot(n,hit_n)>.9) glColor3f(1,1,1);
            else if(j==0) glColor3fv((float*)&rgb);
            else glColor3fv((float*)&rgb_inv);
            glBegin(GL_TRIANGLE_STRIP);
            float vtx[3];
            vtx[k]=.5*pos(j);
            for(int i=0;i<4;i++){
                vtx[(k+1)%3]=+.5*pos(i/2);
                vtx[(k+2)%3]=+.5*pos(i%2);
                glVertex3fv(vtx);
            }
            glEnd();
        }
    }
}

struct rubiks_cube{
    _quat rots[3][3][3];
    int3 idxs[3][3][3];
    char target_axis_en;
    int target_angle;
    uint8_t target_slice;
    static constexpr int steps_max=50;
    int steps=0;
    isec hit;
    rubiks_cube(){
        precompute_idxs();
    }
    int3  &get_idx(int3 idx){return idxs[idx.x][idx.y][idx.z];}
    _quat &get_rot(int3 idx){return rots[idx.x][idx.y][idx.z];}
    int3  read_idx(int3 idx)const{return idxs[idx.x][idx.y][idx.z];}
    _quat read_rot(int3 idx)const{return rots[idx.x][idx.y][idx.z];}
    void precompute_idxs(){
        for(int i=0;i<27;i++){
            int3 idx=line2cube_idx(i);
            get_idx(idx)=get_permuted_idx(idx);
        }
    }
    void move(char axis_en,int angle,uint8_t slice){
        if(steps!=0) return;
        target_axis_en=axis_en;
        target_angle=angle;
        target_slice=slice;
        steps=steps_max;
        precompute_idxs();
    }
    static int2 circ2square_idx(int k){
        k%=8;
        if(k<3) return int2(0,k);
        else if(k==3) return int2(1,2);
        else if(k<7) return int2(2,2-(k-4));
        return int2(1,0);
    }
    int3 get_permuted_idx(int3 idx)const{
        float3 v(idx);
        v-=1;
        v=rotate(v,read_rot(idx));
        v+=1;
        return glm::round(v);
    }
    void move_inc(char axis_en,float angle,uint8_t slice){
        if(angle==0) return;
        float3 axis;
        switch(axis_en){
            case 'x': axis=float3(1,0,0);break;
            case 'y': axis=float3(0,1,0);break;
            case 'z': axis=float3(0,0,1);break;
            default: assert(0);
        }
        int modified=0;

        const _quat nq=angle_axis(pi/2*angle,axis);
        for(int i=0;i<27;i++){
            const int3 idx1=line2cube_idx(i);
            const int3 idx2=read_idx(idx1);
            switch(axis_en){
                case 'x': if(idx2.x!=slice) continue;break;
                case 'y': if(idx2.y!=slice) continue;break;
                case 'z': if(idx2.z!=slice) continue;break;
            }
            _quat &q=get_rot(idx1);
            q=nq*q;
            const int3 nidx=get_permuted_idx(idx1);
            switch(axis_en){
            case 'x': assert(nidx.x==slice);break;
            case 'y': assert(nidx.y==slice);break;
            case 'z': assert(nidx.z==slice);break;
            }
            modified++;
        }
        assert(modified==9);
    }
    void constrain(){
        float max_diff=0;
        for(int i=0;i<27;i++){
            const int3 idx=line2cube_idx(i);
            _quat &q=get_rot(idx);//][idx.y][idx.z];
            float min_diff=1e20;
            _quat closest(1,float3(0));
            for(int j=0;j<24;j++){
                _quat div=q*group[j].conj();
                float diff=abs(div.get_angle());
                if(diff<min_diff){
                    min_diff=diff;
                    closest=group[j];
                }
            }
            max_diff=glm::max(max_diff,min_diff);
            if(min_diff>1){
                lprintln(q);
                lprintln(closest);
                assert(0);
            }
            q=closest;
        }
    }
    void draw(float3 mouse,float3 view){
        if(steps>0){
            float x=target_angle/(float)steps_max;
            move_inc(target_axis_en,x,target_slice);
            steps--;
        }
        glScalef(.7,.7,.7);
        const float s=.425;
        mouse/=float3(.7);
        hit.hit=false;
        hit.t=1e20;
        for(int i=0;i<27;i++){
            int3 idx=line2cube_idx(i);
            _quat q=get_rot(idx);
            float3 mouse_tmp=rotate(mouse,q.conj());
            float3 view_tmp=rotate(view,q.conj());
            float3 p=.5f*(float3(idx)-1.0f);
            mouse_tmp-=p;
            mouse_tmp/=s;
            isec tmp_isec=cube_intersect(mouse_tmp,view_tmp);
            if(tmp_isec.hit && tmp_isec.t<hit.t){
                hit=tmp_isec;
                hit.hit=true;
                hit.idx=idx;
                hit.q=q;
            }

        }
        for(int i=0;i<27;i++){
            int3 idx=line2cube_idx(i);
            glPushMatrix();
            _quat q=get_rot(idx);
            float4x4 mat=q.to_mtx();
            glMultMatrixf((float*)&mat);
            float3 p=.5f*(float3(idx)-1.0f);
            glTranslatef(p.x,p.y,p.z);
            glScalef(s,s,s);
            if(hit.hit && hit.idx==idx) draw_test_cube(true,hit.n);
            else draw_test_cube(false,float3(0));
            glPopMatrix();
        }
        if(hit.hit){
            float3 an=glm::abs(rotate(hit.n,hit.q));
            if(an.x>an.y && an.x>an.z) hit.not_axis_en='x';
            else if(an.y>an.x && an.y>an.z) hit.not_axis_en='y';
            else hit.not_axis_en='z';
            hit.idx_pos=get_idx(hit.idx);
        }
    }
};

void draw_transform(){
    glLineWidth(5);
    glBegin(GL_LINES);
    glColor3f(1,0,0);
    glVertex3f(0,0,0);
    glVertex3f(1,0,0);
    glColor3f(0,1,0);
    glVertex3f(0,0,0);
    glVertex3f(0,1,0);
    glColor3f(0,0,1);
    glVertex3f(0,0,0);
    glVertex3f(0,0,1);
    glEnd();
}


@interface AppDelegate:NSObject<NSApplicationDelegate,NSWindowDelegate>
@property (strong) NSWindow *window;
@property (strong) NSOpenGLContext *ctx;
@property (strong) NSOpenGLView *view;
@property float width;
@property float height;
@property struct{bool down[256];} keys;
@property bool left_mouse_down;
@property bool right_mouse_down;
@property float2 mouse_pos;
@property _quat base_view;
@property _quat tmp_view;
@property float3 base_mouse_vec;
@property rubiks_cube *rc;
@property bool moving;
@property float prev_angle;
@property isec hit;
@property bool axis_resolved;
@property char move_axis_en;
@property char move_slice;
-(void)setup;
-(void)draw;
@end

@implementation AppDelegate
-(float3)get_mouse_vec{
    float aspect=_width/(float)_height;
    return float3(map(_mouse_pos.x,0,_width,-aspect,aspect),
                  map(_mouse_pos.y,0,_height,-1,1),1);
}
-(void)set_left_mouse_down:(bool) down{
//    assert(_left_mouse_down!=down);
    _left_mouse_down=down;
    _prev_angle=0;
    if(_rc->steps==0){
        if(_rc->hit.hit && down){
            assert(!_moving);
            assert(!_axis_resolved);
            _moving=true;
            _base_mouse_vec=[self get_mouse_vec];
            _rc->precompute_idxs();
            _hit=_rc->hit;
            return;
        }
        if(!down && _moving){
            _moving=false;
            _rc->constrain();
            _rc->precompute_idxs();
            _axis_resolved=false;
            return;
        }
    }
    if(!_moving){
        if(down){
            _base_mouse_vec=[self get_mouse_vec];
        }else{
            _base_view*=_tmp_view;
        }
    }

}
-(void)set_right_mouse_down:(bool) down{
    _right_mouse_down=down;
}
-(void)applicationDidFinishLaunching:(NSNotification*)aNotification{
	puts(__func__);
    [self setup];
    [NSApp stop:self];
}
-(void)setup{
    _rc=new rubiks_cube;
    for(int i=0;i<256;i++) _keys.down[i]=false;
    _left_mouse_down=false;
    _right_mouse_down=false;
    _moving=false;
    _prev_angle=0;
    _axis_resolved=false;
    _move_axis_en='x';
    
    _width=640;
    _height=480;
    auto rect=NSMakeRect(0,700,_width,_height);
	_window=[[NSWindow alloc]initWithContentRect:rect
                             styleMask:NSWindowStyleMaskTitled|NSWindowStyleMaskClosable|
                                 NSWindowStyleMaskMiniaturizable|NSWindowStyleMaskResizable
                             backing:NSBackingStoreBuffered
                             defer:false];
	assert(_window);
	[_window setTitle:@"._____."];
    [_window makeKeyAndOrderFront:nil];
    _window.delegate=self;


	uint32_t attribs[]={NSOpenGLPFADoubleBuffer,NSOpenGLPFADepthSize,24,0};
	NSOpenGLPixelFormat *format=[[NSOpenGLPixelFormat alloc]initWithAttributes:attribs];
	assert(format);
    
    _view=[[NSOpenGLView alloc]initWithFrame:rect
                               pixelFormat:format];
    assert(_view);
    _ctx=_view.openGLContext;
	assert(_ctx);

	[_ctx makeCurrentContext];
    assert(_window.contentView);
    [_ctx setView:_window.contentView];
	assert([_ctx view]);
	
    puts((const char*)glGetString(GL_VERSION));
	glEnable(GL_DEPTH_TEST);
    assert(glIsEnabled(GL_DEPTH_TEST));
}
-(void)draw{
    _width =_window.frame.size.width;
    _height=_window.frame.size.height;
    
    _mouse_pos.x=NSEvent.mouseLocation.x-_window.frame.origin.x;
    _mouse_pos.y=NSEvent.mouseLocation.y-_window.frame.origin.y;

    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    
    glViewport(0,0,2*_width,2*_height);
    glDepthMask(GL_TRUE);
    glPushMatrix();
    glBegin(GL_TRIANGLE_STRIP);
    glColor3f(.9,.6,.6);
    glVertex3f(-1,-1,.9);
    glVertex3f(+1,-1,.9);
    glColor3f(.6,.6,.9);
    glVertex3f(-1,+1,.9);
    glVertex3f(+1,+1,.9);
    glEnd();

    
    glScalef(1,1,.1);
    glScalef(_height/_width,1,-1);
    

    _quat cam_q=_base_view;
    float3 mouse_vec=[self get_mouse_vec];
    mouse_vec=rotate(mouse_vec,cam_q.conj());
    float3 mouse_vec2=mouse_vec;
    float3 base_vec=_base_mouse_vec;
    base_vec=rotate(base_vec,cam_q.conj());
    _tmp_view=from_to(base_vec,mouse_vec);
    
    if(_left_mouse_down && !_moving){
        cam_q=_base_view*_tmp_view;
        mouse_vec2=rotate([self get_mouse_vec],cam_q.conj());
    }
    
    auto cam_mtx=cam_q.to_mtx();
    glMultMatrixf((float*)&cam_mtx);
    
    
    if(_moving){
        float3 hit_n=.5f*rotate(_hit.n,_hit.q);
        float3 delta_m=mouse_vec-base_vec;
        float3 np=glm::normalize(hit_n);
        float3 cx=glm::cross(delta_m,-np);
        
        
        float a=cx[_move_axis_en-'x'];
        float angle=3*a/2;
        
        if(!_axis_resolved){
            float max_dot=0;
            for(int i=0;i<3;i++) if(_hit.not_axis_en!='x'+i && abs(cx[i])>abs(max_dot)){
                max_dot=cx[i];
                _move_axis_en='x'+i;
            }
            if(abs(max_dot)>.02){
                _axis_resolved=true;
                _move_slice=_hit.idx_pos[_move_axis_en-'x'];
            }
        }else{
            
            _rc->move_inc(_move_axis_en,angle-_prev_angle,_move_slice);
            _prev_angle=angle;
        }
    }
    
//    draw_transform();

    float3 view_dir=float3(0,0,-1);
    view_dir=rotate(view_dir,cam_q.conj());

    _rc->draw(mouse_vec2,view_dir);
    
    
    glPopMatrix();

    uint32_t e=glGetError();
    if(e!=GL_NO_ERROR){
        print_glerror(e);
        exit(1);
    }
    [_ctx flushBuffer];

}
-(BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication *)sender{
    return true;
}
-(void)applicationWillTerminate:(NSNotification *)notification{
    puts(__func__);
}
-(void)windowWillClose:(NSNotification *)notification{
    puts(__func__);
    [NSApp terminate:self];
}
@end

int main(){
    puts(__func__);
    
	[NSApplication sharedApplication];
	AppDelegate *del=[[AppDelegate alloc]init];
    NSApp.delegate=del;
    [NSApp activateIgnoringOtherApps:YES];
    [NSApp run];
    while(1){
        NSEvent *event=[NSApp nextEventMatchingMask:NSEventMaskAny 
                              untilDate:[NSDate dateWithTimeIntervalSinceNow:1/120.0]
                              inMode:NSEventTrackingRunLoopMode
                              dequeue:YES];

        if(event!=nil){
            if((event.type==NSEventTypeKeyUp || event.type==NSEventTypeKeyDown) && event.isARepeat==NO){
                //NSLog(@"Event: %@", event);
                char c=*event.characters.UTF8String;
                auto keys=del.keys;
                keys.down[c]=event.type==NSEventTypeKeyDown;
                if(event.type==NSEventTypeKeyDown){
                    switch(c){
                        case 'q': del.rc->move('x',1,0);break;
                        case 'w': del.rc->move('x',1,1);break;
                        case 'e': del.rc->move('x',1,2);break;
                        case 'a': del.rc->move('y',1,0);break;
                        case 's': del.rc->move('y',1,1);break;
                        case 'd': del.rc->move('y',1,2);break;
                        case 'z': del.rc->move('z',1,0);break;
                        case 'x': del.rc->move('z',1,1);break;
                        case 'c': del.rc->move('z',1,2);break;
                    }
                }
            }
            switch(event.type){
                case NSEventTypeRightMouseDown: [del set_right_mouse_down:true];break;
                case NSEventTypeRightMouseUp:   [del set_right_mouse_down:false];break;
                case NSEventTypeLeftMouseDown:  [del set_left_mouse_down:true];break;
                case NSEventTypeLeftMouseUp:    [del set_left_mouse_down:false];break;
                default: break;
            }
            [NSApp sendEvent:event];
        }
        [del draw];
//         print_fps();
    }
}



















//
